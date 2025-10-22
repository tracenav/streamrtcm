/**
 * Minimal PPL RTK Base Station for ESP32-S3 QEMU
 * 
 * Simple PPL passthrough that:
 * 1. Initializes PPL library
 * 2. Accepts NMEA/RTCM/SPARTN data via UART
 * 3. Outputs RTK corrections
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "PPL_PublicInterface.h"
#include "PPL_Version.h"
#include "secrets.h"

#define UART_PORT UART_NUM_0
#define UART_RX_BUF 4096
#define UART_TX_BUF 4096

static const char *TAG = "RTK_BASE";

static int parse_decimal(const char *s) {
    int v = 0; 
    while (*s >= '0' && *s <= '9') { 
        v = v * 10 + (*s - '0'); 
        s++; 
    } 
    return v;
}

static void ppl_output_task(void *arg) {
    static uint8_t rtcm_buffer[PPL_MAX_RTCM_BUFFER];
    
    while (1) {
        uint32_t output_length = 0;
        ePPL_ReturnStatus result = PPL_GetRTCMOutput(rtcm_buffer, sizeof(rtcm_buffer), &output_length);
        
        if (result == ePPL_Success && output_length > 0) {
            ESP_LOGI(TAG, "RTK correction: %lu bytes", output_length);
            
            // Output framed RTK data
            printf("!RTK %lu\n", output_length);
            fwrite(rtcm_buffer, 1, output_length, stdout);
            printf("\n");
            fflush(stdout);
        }
        
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void uart_input_task(void *arg) {
    static uint8_t rx_buffer[UART_RX_BUF];
    static char line_buffer[256];
    int line_pos = 0;
    
    enum { IDLE, READING_HEADER, READING_BINARY } state = IDLE;
    char data_type = 0; // 'R' for RTCM, 'S' for SPARTN
    int expected_bytes = 0;
    static uint8_t binary_buffer[PPL_MAX_RTCM_BUFFER];
    int binary_pos = 0;

    while (1) {
        int bytes_read = uart_read_bytes(UART_PORT, rx_buffer, sizeof(rx_buffer), pdMS_TO_TICKS(20));
        if (bytes_read <= 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        for (int i = 0; i < bytes_read; i++) {
            char ch = (char)rx_buffer[i];
            
            if (state == IDLE) {
                if (ch == '$') {
                    // Start of NMEA sentence
                    line_pos = 0;
                    line_buffer[line_pos++] = ch;
                    state = READING_HEADER;
                } else if (ch == '!') {
                    // Start of binary data frame
                    line_pos = 0;
                    state = READING_HEADER;
                }
            } else if (state == READING_HEADER) {
                if (ch == '\n' || ch == '\r') {
                    line_buffer[line_pos] = '\0';
                    
                    if (line_buffer[0] == '$') {
                        // NMEA sentence - send directly to PPL
                        PPL_SendRcvrData(line_buffer, strlen(line_buffer));
                        ESP_LOGI(TAG, "NMEA: %s", line_buffer);
                    } else {
                        // Parse binary frame header: "R <len>" or "S <len>"
                        const char *p = line_buffer;
                        if (*p == '!') p++;
                        while (*p == ' ') p++;
                        
                        data_type = *p++;
                        while (*p == ' ') p++;
                        expected_bytes = parse_decimal(p);
                        
                        if ((data_type == 'R' || data_type == 'S') && 
                            expected_bytes > 0 && 
                            expected_bytes <= (int)sizeof(binary_buffer)) {
                            binary_pos = 0;
                            state = READING_BINARY;
                        } else {
                            state = IDLE;
                        }
                    }
                } else if (line_pos < (int)sizeof(line_buffer) - 1) {
                    line_buffer[line_pos++] = ch;
                } else {
                    state = IDLE; // Line too long, reset
                }
            } else if (state == READING_BINARY) {
                binary_buffer[binary_pos++] = (uint8_t)ch;
                
                if (binary_pos >= expected_bytes) {
                    // Complete binary frame received
                    if (data_type == 'R') {
                        PPL_SendRcvrData((const char *)binary_buffer, expected_bytes);
                        ESP_LOGI(TAG, "RTCM: %d bytes", expected_bytes);
                    } else if (data_type == 'S') {
                        PPL_SendAuxSpartn(binary_buffer, expected_bytes);
                        ESP_LOGI(TAG, "SPARTN: %d bytes", expected_bytes);
                    }
                    state = IDLE;
                }
            }
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "=== Minimal PPL RTK Base Station ===");
    ESP_LOGI(TAG, "Target: ESP32-S3 QEMU");
    ESP_LOGI(TAG, "%s", PPL_SDK_VERSION);
    
    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    
    uart_driver_install(UART_PORT, UART_RX_BUF, UART_TX_BUF, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);
    
    // Initialize PPL
    ePPL_ReturnStatus init_result = PPL_Initialize(PPL_CFG_ENABLE_AUX_CHANNEL);
    ESP_LOGI(TAG, "PPL_Initialize: %d", (int)init_result);
    
    if (init_result != ePPL_Success) {
        ESP_LOGE(TAG, "PPL initialization failed!");
        return;
    }
    
    // Send dynamic key
    ePPL_ReturnStatus key_result = PPL_SendDynamicKey(currentDynamicKey, currentKeyLength);
    ESP_LOGI(TAG, "PPL_SendDynamicKey: %d", (int)key_result);
    
    if (key_result != ePPL_Success) {
        ESP_LOGE(TAG, "PPL key setup failed!");
        return;
    }
    
    ESP_LOGI(TAG, "=== PPL Ready ===");
    ESP_LOGI(TAG, "Send NMEA sentences starting with $");
    ESP_LOGI(TAG, "Send RTCM data as: !R <length>\\n<binary_data>");
    ESP_LOGI(TAG, "Send SPARTN data as: !S <length>\\n<binary_data>");
    
    // Start tasks
    xTaskCreate(ppl_output_task, "ppl_output", 4096, NULL, 5, NULL);
    xTaskCreate(uart_input_task, "uart_input", 6144, NULL, 6, NULL);
    
    ESP_LOGI(TAG, "=== RTK Base Station Running ===");
}
