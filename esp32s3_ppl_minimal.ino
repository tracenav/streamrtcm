/*
  ESP32-S3 + PPL minimal feeder

  Purpose: Bare-bones sketch which:
  - Initializes PPL with dynamic key from secrets.h
  - Reads UART (USB CDC Serial) for three input types:
    1) NMEA lines ($GPGGA / $GPZDA) → PPL_SendRcvrData
    2) RTCM binary frames (framed)   → PPL_SendRcvrData
    3) SPARTN binary frames (framed) → PPL_SendAuxSpartn
  - Polls PPL for RTCM output and reports minimal convergence status

  Framing protocol over Serial (from host Python feeder):
    - NMEA: ASCII line starting with '$' and ending with '\n' or '\r\n'
    - Binary frames (two-step):
        Header line:  !R <len>\n   or   !S <len>\n
          R = RTCM, S = SPARTN, <len> = decimal length of payload bytes
        Then exactly <len> raw bytes
        Then a single '\n' (newline) as a terminator

  This sketch purposely avoids WiFi, sockets, sensors, or extra logging.
  It is intended only to validate PPL convergence with a UART feeder.
*/

#include <Arduino.h>

#include "PPL_PublicInterface.h"
#include "PPL_Version.h"
#include "secrets.h"  // must define: const char* currentDynamicKey; const uint32_t currentKeyLength;

// --- Minimal logging macro (can disable by setting DBG_ON to 0) ---
#define DBG_ON 1
#if DBG_ON
#define DBG(...) Serial.println(__VA_ARGS__)
#else
#define DBG(...)
#endif

// --- UART framing constants ---
static const char FRAME_MARKER = '!';  // header marker
static const char FRAME_TYPE_RTCM = 'R';
static const char FRAME_TYPE_SPARTN = 'S';

// --- Buffers ---
static const size_t NMEA_BUFFER_SIZE = 256;
static const size_t BINARY_BUFFER_SIZE = PPL_MAX_RTCM_BUFFER; // reuse PPL buffer size as a safe upper bound

// Input state machine
enum class InputState : uint8_t {
  Idle,
  ReadingNmea,        // reading until newline
  ReadingHeader,      // reading "!X <len>\n"
  ReadingBinary,      // reading <len> bytes
  ReadingBinaryTail   // reading trailing '\n' after payload
};

static InputState inputState = InputState::Idle;

// NMEA line buffer
static char nmeaBuffer[NMEA_BUFFER_SIZE];
static size_t nmeaIndex = 0;

// Header parsing
static char headerType = 0;  // 'R' or 'S'
static char headerLine[24];
static size_t headerIdx = 0;
static int binaryExpectedLen = 0;

// Binary payload buffer
static uint8_t binaryBuffer[BINARY_BUFFER_SIZE];
static size_t binaryIndex = 0;

// PPL output buffer
static uint8_t pplOutBuffer[PPL_MAX_RTCM_BUFFER];

// Minimal helper: trim spaces in header line and parse len
static bool parseHeaderLine(const char* line, char& typeOut, int& lenOut) {
  // Expected form: "!R <len>" or "!S <len>"
  // line does not include leading '!' (we capture it separately by state)
  // We accept optional spaces
  const char* p = line;
  // Skip spaces
  while (*p == ' ' || *p == '\t') p++;
  if (*p != 'R' && *p != 'S') return false;
  typeOut = *p;
  p++;
  // Skip spaces
  while (*p == ' ' || *p == '\t') p++;
  // Parse decimal length
  long value = 0;
  bool hasDigit = false;
  while (*p >= '0' && *p <= '9') { value = value * 10 + (*p - '0'); hasDigit = true; p++; }
  if (!hasDigit) return false;
  // Optional trailing spaces are allowed
  while (*p == ' ' || *p == '\t') p++;
  if (*p != '\0') return false; // nothing else expected
  if (value < 0 || value > (long)BINARY_BUFFER_SIZE) return false;
  lenOut = (int)value;
  return true;
}

// Feed a complete NMEA sentence to PPL
static void handleNmeaLine(const char* line) {
  // Pass through as-is (without adding CR/LF)
  PPL_SendRcvrData(line, strlen(line));
}

// Feed a complete RTCM frame to PPL
static void handleRtcmFrame(const uint8_t* data, size_t length) {
  PPL_SendRcvrData((const char*)data, (uint32_t)length);
}

// Feed a complete SPARTN frame to PPL
static void handleSpartnFrame(const uint8_t* data, size_t length) {
  (void)PPL_SendAuxSpartn((uint8_t*)data, (uint32_t)length);
}

// Poll PPL for output RTCM and print minimal status
static void pollPplOutput() {
  uint32_t outLen = 0;
  ePPL_ReturnStatus st = PPL_GetRTCMOutput(pplOutBuffer, sizeof(pplOutBuffer), &outLen);
  if (st == ePPL_Success && outLen > 0) {
    // Parse a short summary of RTCM message types contained in the buffer
    String types = "";
    size_t i = 0;
    int printed = 0;
    while (i + 5 < outLen && printed < 8) { // need at least D3 + 2 len + 2 type bytes
      // Find preamble 0xD3
      if (pplOutBuffer[i] != 0xD3) { i++; continue; }
      if (i + 2 >= outLen) break;
      uint16_t len10 = ((uint16_t)pplOutBuffer[i+1] << 8) | (uint16_t)pplOutBuffer[i+2];
      uint16_t msgLen = len10 & 0x03FF; // 10-bit length
      size_t totalLen = 3 + (size_t)msgLen + 3; // header+payload+CRC
      if (i + totalLen > outLen) break;
      // Extract message type: first 12 bits of payload
      if (msgLen >= 2) {
        uint16_t msgType = ((uint16_t)pplOutBuffer[i+3] << 4) | (pplOutBuffer[i+4] >> 4);
        if (types.length() > 0) types += ",";
        types += String(msgType);
        printed++;
      }
      i += totalLen;
    }
    if (types.length() > 0) {
      DBG(String("RTK OUT: ") + String(outLen) + String(" bytes | types: ") + types);
    } else {
      DBG(String("RTK OUT: ") + String(outLen) + String(" bytes"));
    }

    // Send raw RTCM bytes back to host over Serial using a simple frame:
    // !O <len>\n <raw bytes> \n
    Serial.print("!O ");
    Serial.print(outLen);
    Serial.print('\n');
    Serial.write(pplOutBuffer, outLen);
    Serial.print('\n');
  }
}

// Process incoming bytes from Serial using the framing protocol
static void processSerialInput() {
  while (Serial.available() > 0) {
    const int byteVal = Serial.read();
    if (byteVal < 0) return;
    const char ch = (char)byteVal;

    switch (inputState) {
      case InputState::Idle:
        if (ch == '$') {
          // Begin NMEA line
          nmeaIndex = 0;
          nmeaBuffer[nmeaIndex++] = ch;
          inputState = InputState::ReadingNmea;
        } else if (ch == FRAME_MARKER) {
          // Begin header line
          headerIdx = 0;
          headerLine[0] = '\0';
          inputState = InputState::ReadingHeader;
        } else {
          // ignore
        }
        break;

      case InputState::ReadingNmea:
        if (ch == '\n' || ch == '\r') {
          // Terminate string and deliver
          if (nmeaIndex < NMEA_BUFFER_SIZE) {
            nmeaBuffer[nmeaIndex] = '\0';
            handleNmeaLine(nmeaBuffer);
          }
          inputState = InputState::Idle;
        } else {
          if (nmeaIndex + 1 < NMEA_BUFFER_SIZE) {
            nmeaBuffer[nmeaIndex++] = ch;
          } else {
            // Overflow: reset to Idle
            inputState = InputState::Idle;
          }
        }
        break;

      case InputState::ReadingHeader:
        if (ch == '\n' || ch == '\r') {
          // End of header line
          headerLine[headerIdx] = '\0';
          if (parseHeaderLine(headerLine, headerType, binaryExpectedLen) && binaryExpectedLen > 0) {
            binaryIndex = 0;
            inputState = InputState::ReadingBinary;
          } else {
            inputState = InputState::Idle; // malformed header
          }
        } else {
          if (headerIdx + 1 < sizeof(headerLine)) {
            headerLine[headerIdx++] = ch;
          } else {
            inputState = InputState::Idle; // header too long
          }
        }
        break;

      case InputState::ReadingBinary:
        binaryBuffer[binaryIndex++] = (uint8_t)ch;
        if ((int)binaryIndex >= binaryExpectedLen) {
          // Full payload received; next expect a single '\n' tail
          inputState = InputState::ReadingBinaryTail;
        }
        break;

      case InputState::ReadingBinaryTail:
        // Consume exactly one trailing newline if present
        if (ch == '\n' || ch == '\r') {
          // Deliver frame
          if (headerType == FRAME_TYPE_RTCM) {
            handleRtcmFrame(binaryBuffer, (size_t)binaryExpectedLen);
          } else if (headerType == FRAME_TYPE_SPARTN) {
            handleSpartnFrame(binaryBuffer, (size_t)binaryExpectedLen);
          }
        } else {
          // No newline; still deliver to be tolerant
          if (headerType == FRAME_TYPE_RTCM) {
            handleRtcmFrame(binaryBuffer, (size_t)binaryExpectedLen);
          } else if (headerType == FRAME_TYPE_SPARTN) {
            handleSpartnFrame(binaryBuffer, (size_t)binaryExpectedLen);
          }
        }
        inputState = InputState::Idle;
        break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println();
  DBG("ESP32-S3 PPL minimal feeder");
  DBG(PPL_SDK_VERSION);

  // Initialize PPL
  ePPL_ReturnStatus st = PPL_Initialize(PPL_CFG_ENABLE_AUX_CHANNEL);
  DBG(String("PPL_Initialize: ") + String((int)st));

  if (st == ePPL_Success) {
    // currentDynamicKey is a hex string; currentKeyLength is number of hex characters
    st = PPL_SendDynamicKey(currentDynamicKey, currentKeyLength);
    DBG(String("PPL_SendDynamicKey: ") + String((int)st));
  }
}

void loop() {
  processSerialInput();
  pollPplOutput();
}


