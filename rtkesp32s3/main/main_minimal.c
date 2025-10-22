// Bare ESP-IDF app for QEMU: PPL passthrough over UART0
// - Reads NMEA lines and framed RTCM/SPARTN from UART0 (QEMU console)
// - Feeds PPL
// - Emits raw RTCM from PPL back to host using framing: !O <len>\n <bytes> \n
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_timer.h"

#include "PPL_PublicInterface.h"
#include "PPL_Version.h"
#include "secrets.h"

#define UART_PORT UART_NUM_0
#define UART_RX_BUF 16384
#define UART_TX_BUF 16384

static int parse_decimal(const char *s) {
  int v = 0; while (*s >= '0' && *s <= '9') { v = v * 10 + (*s - '0'); s++; } return v;
}

static const char* ppl_status_str(ePPL_ReturnStatus st) {
  switch (st) {
    case ePPL_Success: return "Success";
    case ePPL_IncorrectLibUsage: return "IncorrectLibUsage";
    case ePPL_LibInitFailed: return "LibInitFailed";
    case ePPL_LibExpired: return "LibExpired";
    case ePPL_NoDynamicKey: return "NoDynamicKey";
    case ePPL_FailedDynKeyLibPush: return "DynKeyPushFailed";
    case ePPL_InvalidDynKey: return "InvalidDynKey";
    case ePPL_IncorrectDynKey: return "IncorrectDynKey";
    case ePPL_RcvPosNotAvailable: return "RcvPosNotAvailable";
    case ePPL_LeapSecsNotAvailable: return "LeapSecsNotAvailable";
    case ePPL_AreaDefNotAvailableForPos: return "OutsideCoverage";
    case ePPL_TimeNotResolved: return "TimeNotResolved";
    default: return "Unknown";
  }
}

// Debug counters
static volatile uint32_t dbg_spartn_frames = 0;
static volatile uint32_t dbg_spartn_bytes = 0;
static volatile uint32_t dbg_rtcm_in_frames = 0;
static volatile uint32_t dbg_rtcm_in_bytes = 0;
static volatile uint32_t dbg_nmea_gga = 0;
static volatile uint32_t dbg_nmea_zda = 0;
static volatile uint32_t dbg_ppl_out_frames = 0;
static volatile uint32_t dbg_ppl_out_bytes = 0;
static volatile ePPL_ReturnStatus dbg_last_ppl_status = ePPL_Success;
static volatile uint32_t dbg_dyn_key_len = 0;

static void ppl_poll_task(void *arg) {
  static uint8_t out_buf[PPL_MAX_RTCM_BUFFER];
  uint32_t last_status_ms = 0;
  while (1) {
    uint32_t out_len = 0;
    ePPL_ReturnStatus st = PPL_GetRTCMOutput(out_buf, sizeof(out_buf), &out_len);
    dbg_last_ppl_status = st;
    if (st == ePPL_Success && out_len > 0) {
      // Build RTCM types list similar to Arduino sketch
      char types_buf[128];
      types_buf[0] = '\0';
      size_t i = 0; int printed = 0;
      while (i + 5 < out_len && printed < 8) {
        if (out_buf[i] != 0xD3) { i++; continue; }
        if (i + 3 > out_len) break;
        uint16_t len10 = ((uint16_t)out_buf[i+1] << 8) | (uint16_t)out_buf[i+2];
        uint16_t msg_len = len10 & 0x03FF;
        size_t total = 3 + (size_t)msg_len + 3;
        if (i + total > out_len) break;
        if (msg_len >= 2) {
          uint16_t msg_type = ((uint16_t)out_buf[i+3] << 4) | (out_buf[i+4] >> 4);
          char one[12];
          snprintf(one, sizeof(one), "%s%u", (types_buf[0] ? "," : ""), (unsigned)msg_type);
          strncat(types_buf, one, sizeof(types_buf) - strlen(types_buf) - 1);
          printed++;
        }
        i += total;
      }
      if (types_buf[0]) {
        printf("RTK OUT: %lu bytes | types: %s\n", (unsigned long)out_len, types_buf);
      } else {
        printf("RTK OUT: %lu bytes\n", (unsigned long)out_len);
      }
      fflush(stdout);
      // Emit framed raw data
      char header[32];
      int hl = snprintf(header, sizeof(header), "!O %lu\n", (unsigned long)out_len);
      uart_write_bytes(UART_PORT, header, hl);
      uart_write_bytes(UART_PORT, (const char *)out_buf, out_len);
      uart_write_bytes(UART_PORT, "\n", 1);
      dbg_ppl_out_frames++;
      dbg_ppl_out_bytes += out_len;
    }

    // Periodic status line every ~5s
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if (now_ms - last_status_ms > 5000) {
      last_status_ms = now_ms;
      printf("STATUS: ppl=%s key=%s spartn=%lu/%lu rtcm_in=%lu/%lu nmea(GGA/ZDA)=%lu/%lu ppl_out=%lu/%lu\n",
             ppl_status_str(dbg_last_ppl_status),
             (dbg_dyn_key_len > 0 ? "set" : "unset"),
             (unsigned long)dbg_spartn_frames, (unsigned long)dbg_spartn_bytes,
             (unsigned long)dbg_rtcm_in_frames, (unsigned long)dbg_rtcm_in_bytes,
             (unsigned long)dbg_nmea_gga, (unsigned long)dbg_nmea_zda,
             (unsigned long)dbg_ppl_out_frames, (unsigned long)dbg_ppl_out_bytes);
      fflush(stdout);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

static void feeder_task(void *arg) {
  static uint8_t rx[UART_RX_BUF];
  static char line[128];
  int line_len = 0;
  enum { Idle, ReadingHeader, ReadingBinary, ReadingBinaryTail } state = Idle;
  char header_type = 0; // 'R' or 'S' or 'K'
  int expected_len = 0;
  static uint8_t binbuf[PPL_MAX_RTCM_BUFFER];
  int binpos = 0;

  while (1) {
    int n = uart_read_bytes(UART_PORT, rx, sizeof(rx), pdMS_TO_TICKS(10));
    if (n <= 0) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
    for (int i = 0; i < n; i++) {
      char ch = (char)rx[i];
      if (state == Idle) {
        if (ch == '$') {
          line_len = 0; line[line_len++] = ch; state = ReadingHeader; // reuse to collect line
        } else if (ch == '!') {
          line_len = 0; state = ReadingHeader;
        }
      } else if (state == ReadingHeader) {
        if (ch == '\n' || ch == '\r') {
          line[line_len] = '\0';
          if (line_len > 0 && line[0] == '$') {
            // NMEA line
            PPL_SendRcvrData(line, (uint32_t)strlen(line));
            // Quick classify for debug
            if (strstr(line, ",ZDA,")) dbg_nmea_zda++;
            else if (strstr(line, ",GGA,")) dbg_nmea_gga++;
            state = Idle;
          } else if (line_len > 0) {
            // Expect "R <len>" or "S <len>" (may include leading '!')
            const char *p = line;
            if (*p == '!') p++;
            while (*p == ' ' || *p == '\t') p++;
            header_type = *p; p++;
            while (*p == ' ' || *p == '\t') p++;
            expected_len = parse_decimal(p);
            if ((header_type == 'R' || header_type == 'S' || header_type == 'K') && expected_len > 0 && expected_len <= (int)sizeof(binbuf)) {
              binpos = 0; state = ReadingBinary;
            } else {
              state = Idle;
            }
          } else {
            state = Idle;
          }
        } else {
          if (line_len + 1 < (int)sizeof(line)) line[line_len++] = ch; else state = Idle;
        }
      } else if (state == ReadingBinary) {
        binbuf[binpos++] = (uint8_t)ch;
        if (binpos >= expected_len) state = ReadingBinaryTail;
      } else if (state == ReadingBinaryTail) {
        if (header_type == 'R') {
          ePPL_ReturnStatus r = PPL_SendRcvrData((const char *)binbuf, (uint32_t)expected_len);
          if (r != ePPL_Success) {
            printf("WARN: PPL_SendRcvrData=%s len=%d\n", ppl_status_str(r), expected_len);
            fflush(stdout);
          }
          dbg_rtcm_in_frames++;
          dbg_rtcm_in_bytes += (uint32_t)expected_len;
        } else if (header_type == 'S') {
          // Use PPL_SendSpartn for IP channel (NTRIP), not PPL_SendAuxSpartn
          ePPL_ReturnStatus r = PPL_SendSpartn((uint8_t *)binbuf, (uint32_t)expected_len);
          if (r != ePPL_Success) {
            printf("WARN: PPL_SendSpartn=%s len=%d\n", ppl_status_str(r), expected_len);
            fflush(stdout);
          }
          dbg_spartn_frames++;
          dbg_spartn_bytes += (uint32_t)expected_len;
        } else if (header_type == 'K') {
          ePPL_ReturnStatus r = PPL_SendDynamicKey((const char *)binbuf, (uint32_t)expected_len);
          printf("KEY: PPL_SendDynamicKey: %d (%s) len=%d\n", (int)r, ppl_status_str(r), expected_len);
          fflush(stdout);
          if (r == ePPL_Success) {
            dbg_dyn_key_len = (uint32_t)expected_len;
          }
        }
        state = Idle;
      }
    }
  }
}

void app_main(void) {
  // UART0 (console)
  uart_config_t cfg = {
    .baud_rate = 921600,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT
  };
  uart_driver_install(UART_PORT, UART_RX_BUF, UART_TX_BUF, 0, NULL, 0);
  uart_param_config(UART_PORT, &cfg);

  printf("PPL QEMU bare passthrough\n");
  printf("%s\n", PPL_SDK_VERSION);
  fflush(stdout);

  // Init PPL
  // Use IP channel for NTRIP SPARTN streams (not AUX channel which is for L-band)
  ePPL_ReturnStatus st = PPL_Initialize(PPL_CFG_ENABLE_IP_CHANNEL);
  printf("PPL_Initialize: %d (%s)\n", (int)st, ppl_status_str(st)); fflush(stdout);
  if (st == ePPL_Success) {
    // NTRIP SPARTN streams are typically unencrypted
    printf("PPL ready for IP channel (NTRIP) data\n");
    fflush(stdout);
  }

  // Tasks
  xTaskCreate(ppl_poll_task, "ppl_poll", 4096, NULL, 5, NULL);
  xTaskCreate(feeder_task,   "feeder",    6144, NULL, 6, NULL);
}


