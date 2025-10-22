,,/**********************************************************************
 * PPL RTK Base Station: PX1105R (RTCM) + NEO-D9S (SPARTN) + PPL
 * Raw UBX/Serial implementation for reliable GNSS data - 2025-01-28
 * 
 * Hardware:
 * - PX1105R: RTCM ephemeris messages + GPS time (UART2, pins 5/6)
 * - NEO-D9S: SPARTN correction data via serial (UART1, pins 1/2)
 * - PPL: Processes RTCM ephemeris + SPARTN → RTK corrections
 * - NTRIP: Serves corrections to rovers on port 2101
 *********************************************************************/

 #include <WiFi.h>
 #include <WiFiClient.h>
 #include <WiFiServer.h>
 #include <HardwareSerial.h>
 #include <Wire.h>
 #include <math.h>
 
 #include <SparkFun_u-blox_GNSS_v3.h>  // SparkFun u-blox GNSS library for D9S I2C
 #include "PPL_PublicInterface.h"
 #include "PPL_Version.h"
 #include "secrets.h"           // SPARTN decryption key
 
 /************** USER CONFIG ******************************************/
 const char *WIFI_SSID = "OfficeAASBW";
 const char *WIFI_PASS = "Advance1224!";
 const bool  ENABLE_NTRIP = true;      // Toggle NTRIP caster
 const bool  ENABLE_WEB_DEBUG = false; // Enable web debug server - DISABLED FOR PERFORMANCE
 /*********************************************************************/
 
 /* Simple timestamped debug macro */
 #define DBG_ON 1
 #if DBG_ON
   #define DBG(...) do{                              \
     static char _buf[64];                           \
     sprintf(_buf,"%02lu:%02lu:%02lu.%03lu ",        \
             (millis()/3600000)%24,                  \
             (millis()/60000)%60,                   \
             (millis()/1000)%60,                   \
             millis()%1000);                        \
     Serial.print(_buf); Serial.println(__VA_ARGS__);}while(0)
 #else
   #define DBG(...)
 #endif
 
 /* ---------- Hardware definitions -------------------------------- */
 // PX1105R GNSS module
 #define PX_RX_PIN 4    // ESP32 RX <- PX1105R TX
 #define PX_TX_PIN 5    // ESP32 TX -> PX1105R RX
 #define PX_BAUD 115200
 
 // NEO-D9S L-Band receiver (I2C)
 #define D9S_I2C_ADDRESS 0x43  // NEO-D9S default I2C address
 #define SDA 8
 #define SCL 9
 
 // L-Band frequency for US service
 const uint32_t LBAND_FREQ = 1556290000; // US 1.8 GHz
 
 /* ---------- Objects and globals --------------------------------- */
 HardwareSerial pxSerial(2);   // PX1105R on UART2
 SFE_UBLOX_GNSS myD9S;         // NEO-D9S on I2C
 
 WiFiServer   ntripServer(2101);
 WiFiServer   webServer(80);
 WiFiClient   ntripClient;
 
 // Message counters
 volatile uint32_t cntRTCM1019 = 0, cntRTCM1020 = 0, cntRTCM1042 = 0, cntRTCM1046 = 0;
 volatile uint32_t cntRTCM1005 = 0, cntSPARTN = 0, cntPMP = 0;
 volatile uint32_t cntGGA = 0, cntZDA = 0;
 
 // Satellite tracking for ephemeris messages
 struct SatelliteInfo {
   uint8_t gps_sats[32];     // GPS PRNs (1-32)
   uint8_t glo_sats[24];     // GLONASS slot numbers (1-24)  
   uint8_t gal_sats[36];     // Galileo PRNs (1-36)
   uint8_t bds_sats[63];     // BeiDou PRNs (1-63)
   uint8_t gps_count = 0;
   uint8_t glo_count = 0;
   uint8_t gal_count = 0;
   uint8_t bds_count = 0;
   uint32_t last_update = 0;
 } activeSats;
 
 // GPS Time structure (from rtcmdebug.ino)
 struct GPSTime {
   uint32_t timeOfWeek;      // GPS time of week in milliseconds
   uint32_t subTimeOfWeek;   // Sub-millisecond time in nanoseconds
   uint16_t weekNumber;      // GPS week number
   uint8_t defaultLeapSec;   // Default GPS/UTC leap seconds
   uint8_t currentLeapSec;   // Current GPS/UTC leap seconds
   bool valid;               // Time data is valid
 } gpsTime = {0};
 
 // RTCM 1005 data structure (from rtcmdebug.ino)
 struct RTCM1005Data {
   uint16_t stationId;
   double ecefX, ecefY, ecefZ;
   double latitude, longitude, height;
   bool valid;
   uint32_t timestamp;
 } rtcm1005Data = {0};
 
 // Base station management
 String baseStationGGA = "";
 String currentGGA = "";
 String currentZDA = "";
 bool baseStationSet = false;
 
 // PPL and NTRIP stats
 uint32_t totalRTCMBytes = 0;
 uint32_t rtcmMessageCount = 0;
 
 // RTCM caching for 1Hz NTRIP transmission
 static uint8_t cachedRTCMBuffer[PPL_MAX_RTCM_BUFFER];
 static uint32_t cachedRTCMLength = 0;
 static unsigned long lastRTCMCache = 0;
 static unsigned long lastNTRIPTransmission = 0;
 static uint32_t ntripTransmissionCount = 0;
 
 // D9S connection status
 bool d9sConnected = false;
 
 // PPL data flow management (like mosaic-X5)
 bool newDataSentToPPL = false;
 
 // Define M_PI if not available
 #ifndef M_PI
 #define M_PI 3.14159265358979323846
 #endif
 
 /* ---------- Forward declarations -------------------------------- */
 void setupPX1105R();
 void setupD9S();
 void setupPPL();
 void parsePXData();
 void parseD9SData();
 void generateNMEAFromRTCM();
 void handleNTRIP();
 void handleWebDebug();
 
 /* ---------- PX1105R Functions (RTCM + GPS Time) ---------------- */
 
 // Buffer for PX1105R data
 #define PX_BUFFER_SIZE 4096
 uint8_t pxBuffer[PX_BUFFER_SIZE];
 uint16_t pxBufferIndex = 0;
 
 // GPS time query timing
 unsigned long lastGpsTimeQuery = 0;
 const unsigned long gpsTimeQueryInterval = 1000; // Query every second
 
 // Calculate PX1105R binary checksum
 uint8_t calculatePXChecksum(uint8_t* payload, uint16_t length) {
   uint8_t checksum = 0;
   for (uint16_t i = 0; i < length; i++) {
     checksum ^= payload[i];
   }
   return checksum;
 }
 
 // Send GPS time query to PX1105R
 void queryGpsTime() {
   uint8_t payload[] = {0x64, 0x20};  // GPS Time Query
   uint8_t checksum = calculatePXChecksum(payload, 2);
   uint8_t command[] = {0xA0, 0xA1, 0x00, 0x02, 0x64, 0x20, checksum, 0x0D, 0x0A};
   
   pxSerial.write(command, sizeof(command));
   
   // Removed frequent GPS query logging for performance
 }
 
 // Parse GPS time response from PX1105R
 void parseGpsTimeResponse(uint8_t* data) {
   // GPS time response payload structure (from rtcmdebug.ino)
   gpsTime.timeOfWeek = ((uint32_t)data[2] << 24) | ((uint32_t)data[3] << 16) | 
                        ((uint32_t)data[4] << 8) | data[5];
   
   gpsTime.subTimeOfWeek = ((uint32_t)data[6] << 24) | ((uint32_t)data[7] << 16) | 
                           ((uint32_t)data[8] << 8) | data[9];
   
   gpsTime.weekNumber = ((uint16_t)data[10] << 8) | data[11];
   gpsTime.defaultLeapSec = data[12];
   gpsTime.currentLeapSec = data[13];
   gpsTime.valid = true;
   
   // Debug GPS time response occasionally (reduce output frequency)
   static uint32_t lastTimeLog = 0;
   if (millis() - lastTimeLog > 10000) {  // Every 10 seconds instead of every second
     lastTimeLog = millis();
     DBG("GPS Time Response: Week=" + String(gpsTime.weekNumber) + 
         " TOW=" + String(gpsTime.timeOfWeek/1000) + "s LeapSec=" + String(gpsTime.currentLeapSec));
     
     // Convert to human readable time for verification
     uint32_t seconds = gpsTime.timeOfWeek / 1000;
     uint32_t hours = seconds / 3600;
     uint32_t minutes = (seconds % 3600) / 60;
     uint32_t secs = seconds % 60;
     DBG("GPS Time: " + String(hours) + ":" + String(minutes) + ":" + String(secs) + " UTC");
   }
 }
 
 // Extract bits from RTCM payload (from rtcmdebug.ino)
 int64_t extractSignedBits(uint8_t* data, int startBit, int numBits) {
   uint64_t result = 0;
   
   for (int i = 0; i < numBits; i++) {
     int byteIndex = (startBit + i) / 8;
     int bitIndex = 7 - ((startBit + i) % 8);
     
     if (data[byteIndex] & (1 << bitIndex)) {
       result |= (1ULL << (numBits - 1 - i));
     }
   }
   
   if (result & (1ULL << (numBits - 1))) {
     uint64_t signExtendMask = ~((1ULL << numBits) - 1);
     result |= signExtendMask;
   }
   
   return (int64_t)result;
 }
 
 uint64_t extractUnsignedBits(uint8_t* data, int startBit, int numBits) {
   uint64_t result = 0;
   
   for (int i = 0; i < numBits; i++) {
     int byteIndex = (startBit + i) / 8;
     int bitIndex = 7 - ((startBit + i) % 8);
     
     if (data[byteIndex] & (1 << bitIndex)) {
       result |= (1ULL << (numBits - 1 - i));
     }
   }
   
   return result;
 }
 
 // RTKLIB-style ECEF to LLH conversion
 void convertECEFtoLLH_RTKLIB(double x, double y, double z, double* lat, double* lon, double* height) {
   const double RE_WGS84 = 6378137.0;           // Semi-major axis
   const double FE_WGS84 = 1.0 / 298.257223563; // Flattening
   
   double e2 = FE_WGS84 * (2.0 - FE_WGS84);
   double r2 = x * x + y * y;
   
   *lon = (r2 > 1E-12) ? atan2(y, x) : 0.0;
   *lon *= 180.0 / M_PI;
   
   double zk = 0.0, v = RE_WGS84;
   for (double zval = z; abs(zval - zk) >= 1E-4; ) {
     zk = zval;
     double sinp = zval / sqrt(r2 + zval * zval);
     v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);
     zval = z + v * e2 * sinp;
   }
   
   if (r2 > 1E-12) {
     *lat = atan(zk / sqrt(r2)) * 180.0 / M_PI;
   } else {
     *lat = (z > 0.0) ? 90.0 : -90.0;
   }
   
   *height = sqrt(r2 + zk * zk) - v;
 }
 
 // Parse RTCM 1005 message with correct RTKLIB bit positioning
 void parseRTCM1005(uint8_t* rawData, uint16_t length) {
   if (length < 22) {
     rtcm1005Data.valid = false;
     return;
   }
   
   uint8_t* payload = &rawData[3];  // Skip D3 00 13 header
   
   // Verify message type
   uint16_t msgType = extractUnsignedBits(payload, 0, 12);
   if (msgType != 1005) {
     rtcm1005Data.valid = false;
     return;
   }
   
   // Parse with correct RTKLIB bit positioning
   int bitPos = 12; // Skip message type
   
   rtcm1005Data.stationId = extractUnsignedBits(payload, bitPos, 12);
   bitPos += 12;
   
   uint8_t itrf = extractUnsignedBits(payload, bitPos, 6); // ITRF field
   bitPos += 6;
   
   bitPos += 4; // Skip reserved
   
   // ECEF coordinates
   int64_t ecefX_raw = extractSignedBits(payload, bitPos, 38);
   rtcm1005Data.ecefX = ecefX_raw * 0.0001;
   bitPos += 40; // 38 + 2 reserved
   
   int64_t ecefY_raw = extractSignedBits(payload, bitPos, 38);
   rtcm1005Data.ecefY = ecefY_raw * 0.0001;
   bitPos += 40; // 38 + 2 reserved
   
   int64_t ecefZ_raw = extractSignedBits(payload, bitPos, 38);
   rtcm1005Data.ecefZ = ecefZ_raw * 0.0001;
   
   // Sanity check
   double magnitude = sqrt(rtcm1005Data.ecefX * rtcm1005Data.ecefX + 
                          rtcm1005Data.ecefY * rtcm1005Data.ecefY + 
                          rtcm1005Data.ecefZ * rtcm1005Data.ecefZ);
   
   if (magnitude < 6000000 || magnitude > 7000000) {
     rtcm1005Data.valid = false;
     return;
   }
   
   // Convert to LLH
   convertECEFtoLLH_RTKLIB(rtcm1005Data.ecefX, rtcm1005Data.ecefY, rtcm1005Data.ecefZ,
                          &rtcm1005Data.latitude, &rtcm1005Data.longitude, &rtcm1005Data.height);
   
   rtcm1005Data.valid = true;
   rtcm1005Data.timestamp = millis();
   
   // Set base station if not already set
   if (!baseStationSet) {
     baseStationSet = true;
     DBG("Base station coordinates set from RTCM 1005: " + 
         String(rtcm1005Data.latitude, 8) + "°, " + 
         String(rtcm1005Data.longitude, 8) + "°, " + 
         String(rtcm1005Data.height, 3) + "m");
     
     // Check if coordinates are within PointPerfect coverage (US/CONUS)
     if (rtcm1005Data.latitude >= 25.0 && rtcm1005Data.latitude <= 50.0 && 
         rtcm1005Data.longitude >= -125.0 && rtcm1005Data.longitude <= -65.0) {
       DBG("✓ Position is within PointPerfect US coverage area");
     } else {
       DBG("⚠️ WARNING: Position may be outside PointPerfect coverage area");
       DBG("   US coverage: 25°-50°N, 125°-65°W");
     }
   }
 }
 
 // Extract satellite PRN/slot from RTCM ephemeris messages
 uint8_t extractSatellitePRN(uint16_t messageType, uint8_t* data) {
   if (!data) return 0;
   
   uint8_t* payload = &data[3]; // Skip RTCM header (D3 + length)
   
   switch (messageType) {
     case 1019: // GPS ephemeris - PRN at bits 12-17 (6 bits)
       return extractUnsignedBits(payload, 12, 6);
       
     case 1020: // GLONASS ephemeris - slot number at bits 12-17 (6 bits)  
       return extractUnsignedBits(payload, 12, 6);
       
     case 1042: // BeiDou ephemeris - PRN at bits 12-17 (6 bits)
       return extractUnsignedBits(payload, 12, 6);
       
     case 1046: // Galileo ephemeris - PRN at bits 12-17 (6 bits)
       return extractUnsignedBits(payload, 12, 6);
       
     default:
       return 0;
   }
 }
 
 // Update satellite tracking arrays
 void updateSatelliteList(uint16_t messageType, uint8_t satId) {
   if (satId == 0) return;
   
   switch (messageType) {
     case 1019: // GPS
       if (satId <= 32) {
         // Check if satellite already in list
         bool found = false;
         for (int i = 0; i < activeSats.gps_count; i++) {
           if (activeSats.gps_sats[i] == satId) {
             found = true;
             break;
           }
         }
         if (!found && activeSats.gps_count < 32) {
           activeSats.gps_sats[activeSats.gps_count++] = satId;
         }
       }
       break;
       
     case 1020: // GLONASS  
       if (satId <= 24) {
         bool found = false;
         for (int i = 0; i < activeSats.glo_count; i++) {
           if (activeSats.glo_sats[i] == satId) {
             found = true;
             break;
           }
         }
         if (!found && activeSats.glo_count < 24) {
           activeSats.glo_sats[activeSats.glo_count++] = satId;
         }
       }
       break;
       
     case 1042: // BeiDou
       if (satId <= 63) {
         bool found = false;
         for (int i = 0; i < activeSats.bds_count; i++) {
           if (activeSats.bds_sats[i] == satId) {
             found = true;
             break;
           }
         }
         if (!found && activeSats.bds_count < 63) {
           activeSats.bds_sats[activeSats.bds_count++] = satId;
         }
       }
       break;
       
     case 1046: // Galileo
       if (satId <= 36) {
         bool found = false;
         for (int i = 0; i < activeSats.gal_count; i++) {
           if (activeSats.gal_sats[i] == satId) {
             found = true;
             break;
           }
         }
         if (!found && activeSats.gal_count < 36) {
           activeSats.gal_sats[activeSats.gal_count++] = satId;
         }
       }
       break;
   }
   
   activeSats.last_update = millis();
 }
 
 // Count and process RTCM messages with satellite tracking
 void countRTCMMessage(uint16_t messageType, uint8_t* data, uint16_t length) {
   switch (messageType) {
     case 1005:
       cntRTCM1005++;
       parseRTCM1005(data, length);
       break;
     case 1019: 
       cntRTCM1019++;
       {
         uint8_t satPRN = extractSatellitePRN(messageType, data);
         updateSatelliteList(messageType, satPRN);
         // Send GPS ephemeris to PPL
         PPL_SendRcvrData((const char*)data, length);
         newDataSentToPPL = true;
       }
       break;
     case 1020:
       cntRTCM1020++;
       {
         uint8_t satSlot = extractSatellitePRN(messageType, data);
         updateSatelliteList(messageType, satSlot);
         // Send GLONASS ephemeris to PPL  
         PPL_SendRcvrData((const char*)data, length);
         newDataSentToPPL = true;
       }
       break;
     case 1042:
       cntRTCM1042++;
       {
         uint8_t satPRN = extractSatellitePRN(messageType, data);
         updateSatelliteList(messageType, satPRN);
         // Send BeiDou ephemeris to PPL
         PPL_SendRcvrData((const char*)data, length);
         newDataSentToPPL = true;
       }
       break;
     case 1046:
       cntRTCM1046++;
       {
         uint8_t satPRN = extractSatellitePRN(messageType, data);
         updateSatelliteList(messageType, satPRN);
         // Send Galileo ephemeris to PPL
         PPL_SendRcvrData((const char*)data, length);
         newDataSentToPPL = true;
       }
       break;
     default:
       // Log other message types occasionally
       static uint32_t lastOtherLog = 0;
       if (millis() - lastOtherLog > 10000) {
         lastOtherLog = millis();
         DBG("RTCM " + String(messageType) + " received (" + String(length) + " bytes)");
       }
       break;
   }
 }
 
 /* ---------- NEO-D9S Functions (L-Band SPARTN) - SparkFun Library ------------------ */
 
 // Callback: processRXMPMP called when PMP data arrives from D9S
 void processRXMPMP(UBX_RXM_PMP_message_data_t *pmpData) {
   // Extract the raw message payload length
   uint16_t payloadLen = ((uint16_t)pmpData->lengthMSB << 8) | (uint16_t)pmpData->lengthLSB;
   
   cntPMP++;
   
   // Extract signal quality info
   uint16_t numBytesUserData = pmpData->payload[2] | ((uint16_t)pmpData->payload[3] << 8);
   uint16_t fecBits = pmpData->payload[20] | ((uint16_t)pmpData->payload[21] << 8);
   float ebno = (float)pmpData->payload[22] / 8.0;
   
   // Log PMP messages occasionally (reduce frequency for performance)
   static uint32_t lastPMPLog = 0;
   if (millis() - lastPMPLog > 5000) {  // Every 5 seconds instead of every message
     lastPMPLog = millis();
     DBG("UBX-RXM-PMP len=" + String(payloadLen) + " bytes, Eb/N0=" + String(ebno, 1) + " dB, UserData=" + String(numBytesUserData) + " bytes, fecBits:" + String(fecBits));
   }
   
   // Log detailed signal quality occasionally
   static uint32_t lastQualityLog = 0;
   if (millis() - lastQualityLog > 30000) {  // Every 30 seconds
     lastQualityLog = millis();
     DBG("D9S Detailed - UserBytes:" + String(numBytesUserData) + 
         " FecBits:" + String(fecBits) + 
         " Eb/N0:" + String(ebno, 1) + "dB");
     
     if (ebno < 6.0) {
       DBG("WARNING: Low Eb/N0 signal quality - check antenna");
     } else {
       DBG("Good L-Band signal quality - SPARTN data available");
     }
   }
   
   // Parse the SPARTN data stream contained in the userData using SparkFun library
   for (uint16_t i = 0; i < numBytesUserData; i++) {
     bool valid = false;
     uint16_t len;
     uint8_t *spartn = myD9S.parseSPARTN(pmpData->payload[24 + i], valid, len);
 
     if (valid) {
       cntSPARTN++;
       
              // Send SPARTN data to PPL
        ePPL_ReturnStatus result = PPL_SendAuxSpartn(spartn, len);
        if (result == ePPL_Success) {
          newDataSentToPPL = true;  // Only set flag on success
        } else {
          // SPARTN error logging reduced for performance
          static uint32_t lastSpartnError = 0;
          if (millis() - lastSpartnError > 30000) {  // Only log errors every 30 seconds
            lastSpartnError = millis();
            DBG("PPL_SendAuxSpartn error: " + String((int)result) + " (" + String(PPLReturnStatusToStr(result)) + ")");
          }
        }
       
              // SPARTN logging removed for maximum performance
     }
   }
 }
 
 // Setup D9S module using SparkFun library
 void setupD9S() {
   DBG("=== INITIALIZING NEO-D9S ===");
   
   // Initialize I2C with custom SDA/SCL pins
   Wire.begin(SDA, SCL);
   
   // Connect to NEO-D9S at I2C address 0x43
   while (myD9S.begin(Wire, D9S_I2C_ADDRESS) == false) {
     DBG("u-blox NEO-D9S not detected at I2C address 0x43. Retrying...");
     delay(1000);
   }
   DBG("u-blox NEO-D9S connected via I2C");
   
   // Configure D9S using SparkFun library methods
   DBG("=== CONFIGURING D9S L-BAND ===");
   
   myD9S.newCfgValset(); // Create new configuration message (RAM and BBR layers)
   myD9S.addCfgValset(UBLOX_CFG_PMP_CENTER_FREQUENCY,     LBAND_FREQ);  // US: 1556290000 Hz
   myD9S.addCfgValset(UBLOX_CFG_PMP_SEARCH_WINDOW,        2200);        // Default 2200 Hz
   myD9S.addCfgValset(UBLOX_CFG_PMP_USE_SERVICE_ID,       0);           // Disable service ID filtering
   myD9S.addCfgValset(UBLOX_CFG_PMP_SERVICE_ID,           21845);       // US service ID
   myD9S.addCfgValset(UBLOX_CFG_PMP_DATA_RATE,            2400);        // 2400 bps
   myD9S.addCfgValset(UBLOX_CFG_PMP_USE_DESCRAMBLER,      1);           // Enable descrambler
   myD9S.addCfgValset(UBLOX_CFG_PMP_DESCRAMBLER_INIT,     26969);       // Descrambler init value
   myD9S.addCfgValset(UBLOX_CFG_PMP_USE_PRESCRAMBLING,    0);           // Disable prescrambling
   myD9S.addCfgValset(UBLOX_CFG_PMP_UNIQUE_WORD,          16238547128276412563ULL); // Unique word pattern
   myD9S.addCfgValset(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_I2C,   1);           // Enable UBX-RXM-PMP on I2C
   bool ok = myD9S.sendCfgValset(); // Apply the settings
   
   if (ok) {
     DBG("D9S L-Band configuration successful");
     myD9S.softwareResetGNSSOnly(); // Restart to apply settings
     delay(2000); // Wait for restart
     
     // Set up the callback for PMP messages
     myD9S.setRXMPMPmessageCallbackPtr(&processRXMPMP);
     d9sConnected = true;
     DBG("D9S setup complete - listening for L-Band SPARTN data");
   } else {
     DBG("ERROR: D9S L-Band configuration failed");
     d9sConnected = false;
   }
 }
 
 // Setup PX1105R module
 void setupPX1105R() {
   DBG("=== CONFIGURING PX1105R ===");
   
   pxSerial.begin(PX_BAUD, SERIAL_8N1, PX_RX_PIN, PX_TX_PIN);
   delay(500);
   
   DBG("PX1105R initialized on UART2 at " + String(PX_BAUD) + " baud");
   DBG("Expected RTCM messages: 1005, 1019, 1020, 1042, 1046");
   DBG("GPS time queries will be sent every second");
   DBG("PX1105R setup complete");
 }
 
 /* ---------- PPL Integration ------------------------------------ */
 
 // Helper function for PPL error messages
 const char *PPLReturnStatusToStr(ePPL_ReturnStatus status) {
   switch (status) {
     case ePPL_Success: return "Success";
     case ePPL_IncorrectLibUsage: return "Incorrect library usage";
     case ePPL_LibInitFailed: return "Library initialization failed";
     case ePPL_LibExpired: return "Library expired";
     case ePPL_NoDynamicKey: return "No valid dynamic key";
     case ePPL_FailedDynKeyLibPush: return "Key push failed";
     case ePPL_InvalidDynKey: return "Invalid key format";
     case ePPL_IncorrectDynKey: return "Incorrect key";
     case ePPL_RcvPosNotAvailable: return "Position not available";
     case ePPL_LeapSecsNotAvailable: return "Leap seconds not available";
     case ePPL_AreaDefNotAvailableForPos: return "Outside coverage area";
     case ePPL_TimeNotResolved: return "Time not resolved";
     default: return "Unknown error";
   }
 }
 
 // Setup PPL library
 void setupPPL() {
   DBG("=== INITIALIZING PPL ===");
   
   ePPL_ReturnStatus result = PPL_Initialize(PPL_CFG_ENABLE_AUX_CHANNEL);
   DBG("PPL_Initialize: " + String(PPLReturnStatusToStr(result)));
   
   if (result == ePPL_Success) {
     // PPL expects the hex string directly, not binary conversion
     result = PPL_SendDynamicKey(currentDynamicKey, currentKeyLength);
     DBG("PPL_SendDynamicKey: " + String(PPLReturnStatusToStr(result)));
     
     if (result == ePPL_Success) {
       DBG("PPL initialization successful");
       DBG("Key: " + String(currentDynamicKey).substring(0, 16) + "... (length: " + String(currentKeyLength) + " hex chars)");
     } else {
       DBG("ERROR: PPL key validation failed - check secrets.h");
       DBG("Verify key format: \"6f49600eb72527e0ec592bfbf42bd12f\"");
     }
   } else {
     DBG("ERROR: PPL initialization failed");
   }
 }
 
 /* ---------- NMEA Generation from RTCM 1005 + GPS Time --------- */
 
 // Calculate NMEA checksum
 uint8_t calculateNMEAChecksum(const char* sentence) {
   uint8_t checksum = 0;
   int i = 1; // Start after $
   
   while (sentence[i] != '*' && sentence[i] != '\0') {
     checksum ^= sentence[i];
     i++;
   }
   
   return checksum;
 }
 
 // Generate NMEA GGA from RTCM 1005 + GPS time
 void generateNMEAGGA(char* output, int maxLen) {
   if (!gpsTime.valid || !rtcm1005Data.valid) {
     strcpy(output, "$GPGGA,,,,,,0,00,99.99,,,,,,*");
     return;
   }
   
   // Convert GPS time to UTC
   uint32_t totalSeconds = gpsTime.timeOfWeek / 1000;
   uint32_t hours = (totalSeconds / 3600) % 24;
   uint32_t minutes = (totalSeconds / 60) % 60;
   uint32_t seconds = totalSeconds % 60;
   uint32_t milliseconds = gpsTime.timeOfWeek % 1000;
   
   // Convert coordinates to DDMM.MMMM format
   double latDeg = abs(rtcm1005Data.latitude);
   int latDegrees = (int)latDeg;
   double latMinutes = (latDeg - latDegrees) * 60.0;
   char latDir = (rtcm1005Data.latitude >= 0) ? 'N' : 'S';
   
   double lonDeg = abs(rtcm1005Data.longitude);  
   int lonDegrees = (int)lonDeg;
   double lonMinutes = (lonDeg - lonDegrees) * 60.0;
   char lonDir = (rtcm1005Data.longitude >= 0) ? 'E' : 'W';
   
   // Format the GGA sentence
   snprintf(output, maxLen, 
     "$GPGGA,%02lu%02lu%02lu.%02lu,%02d%08.5f,%c,%03d%08.5f,%c,1,08,1.0,%.3f,M,0.0,M,,",
     hours, minutes, seconds, milliseconds/10,
     latDegrees, latMinutes, latDir,
     lonDegrees, lonMinutes, lonDir,
     rtcm1005Data.height
   );
   
   // Add checksum
   uint8_t checksum = calculateNMEAChecksum(output);
   int len = strlen(output);
   snprintf(&output[len], maxLen - len, "*%02X", checksum);
 }
 
 // Convert GPS week and time to calendar date
 void gpsWeekToCalendar(uint16_t gpsWeek, uint32_t timeOfWeek, int* year, int* month, int* day) {
   // GPS epoch: January 6, 1980 (Sunday)
   const long GPS_EPOCH_JULIAN = 2444244; // Julian day number for Jan 6, 1980
   
   // Calculate days since GPS epoch
   long daysSinceEpoch = gpsWeek * 7 + (timeOfWeek / 1000) / 86400;
   
   // Convert to Julian day number
   long julianDay = GPS_EPOCH_JULIAN + daysSinceEpoch;
   
   // Convert Julian day to calendar date
   long a = julianDay + 32044;
   long b = (4 * a + 3) / 146097;
   long c = a - (146097 * b) / 4;
   long d = (4 * c + 3) / 1461;
   long e = c - (1461 * d) / 4;
   long m = (5 * e + 2) / 153;
   
   *day = e - (153 * m + 2) / 5 + 1;
   *month = m + 3 - 12 * (m / 10);
   *year = 100 * b + d - 4800 + m / 10;
 }
 
 // Generate NMEA ZDA from GPS time with proper date conversion
 void generateNMEAZDA(char* output, int maxLen) {
   if (!gpsTime.valid) {
     strcpy(output, "$GPZDA,,,,,,*");
     return;
   }
   
   // Convert GPS time to UTC time
   uint32_t totalSeconds = gpsTime.timeOfWeek / 1000;
   uint32_t hours = (totalSeconds / 3600) % 24;
   uint32_t minutes = (totalSeconds / 60) % 60;
   uint32_t seconds = totalSeconds % 60;
   uint32_t milliseconds = gpsTime.timeOfWeek % 1000;
   
   // Convert GPS week to calendar date
   int day, month, year;
   gpsWeekToCalendar(gpsTime.weekNumber, gpsTime.timeOfWeek, &year, &month, &day);
   
   // Format the ZDA sentence
   snprintf(output, maxLen,
     "$GPZDA,%02lu%02lu%02lu.%02lu,%02d,%02d,%04d,00,00",
     hours, minutes, seconds, milliseconds/10,
     day, month, year
   );
   
   // Add checksum
   uint8_t checksum = calculateNMEAChecksum(output);
   int len = strlen(output);
   snprintf(&output[len], maxLen - len, "*%02X", checksum);
 }
 
 // Generate and send NMEA to PPL (increased frequency for faster PPL response)
 void generateNMEAFromRTCM() {
   static uint32_t lastNMEAGeneration = 0;
   if (millis() - lastNMEAGeneration < 200) return; // Generate at 5Hz for faster PPL response
   lastNMEAGeneration = millis();
   
   // Debug NMEA generation timing (temporary for diagnosis)
   static uint32_t nmeaCallCount = 0;
   nmeaCallCount++;
   if (nmeaCallCount % 100 == 0) {
     DBG("NMEA generation called " + String(nmeaCallCount) + " times, GPS valid: " + String(gpsTime.valid) + ", RTCM1005 valid: " + String(rtcm1005Data.valid));
   }
   
   if (!gpsTime.valid || !rtcm1005Data.valid) return;
   
   char ggaBuffer[200];
   char zdaBuffer[100];
   
   generateNMEAGGA(ggaBuffer, sizeof(ggaBuffer));
   generateNMEAZDA(zdaBuffer, sizeof(zdaBuffer));
   
   // Send to PPL as combined message for efficiency (mark that new data was sent)
   char combinedNMEA[400];  // Buffer for both messages
   snprintf(combinedNMEA, sizeof(combinedNMEA), "%s\r\n%s\r\n", ggaBuffer, zdaBuffer);
   PPL_SendRcvrData(combinedNMEA, strlen(combinedNMEA));
   newDataSentToPPL = true;  // Signal that new data was sent
   
   // Update counters and store for web interface
   cntGGA++;
   cntZDA++;
   currentGGA = String(ggaBuffer);
   currentZDA = String(zdaBuffer);
   
   // Set base station GGA for NTRIP if not set
   if (!baseStationSet && rtcm1005Data.valid) {
     baseStationGGA = String(ggaBuffer);
     baseStationSet = true;
   }
   
   // NMEA generation logging removed for performance
 }
 
 /* ---------- PPL RTCM Output Handling - Mosaic-X5 Style --------------------------- */
 
 void checkPPLOutput() {
   // Check PPL aggressively - every loop iteration when data sent or every 20ms minimum
   static uint32_t lastPPLCheck = 0;
   bool shouldCheck = newDataSentToPPL || (millis() - lastPPLCheck > 20); // Check every 20ms for maximum responsiveness
   
   if (!shouldCheck) return;
   
   newDataSentToPPL = false; // Reset flag
   lastPPLCheck = millis();
   
   // Check for new RTCM data from PPL
   static uint8_t rtcmBuffer[PPL_MAX_RTCM_BUFFER];
   uint32_t len;
   ePPL_ReturnStatus rtcmStatus = PPL_GetRTCMOutput(rtcmBuffer, PPL_MAX_RTCM_BUFFER, &len);
   
   // Debug PPL RTCM output status occasionally (reduced frequency for performance)
   static uint32_t lastPPLStatusLog = 0;
   if (millis() - lastPPLStatusLog > 30000) {  // Every 30 seconds (reduced from 15)
     lastPPLStatusLog = millis();
     DBG("PPL RTCM Status: " + String(PPLReturnStatusToStr(rtcmStatus)) + " len=" + String(len));
     
     if (rtcmStatus != ePPL_Success) {
       DBG("PPL Issue: " + String((int)rtcmStatus) + " - Check coverage area, key validity, or convergence time");
     } else if (len == 0) {
       DBG("PPL Status: All inputs received, waiting for convergence (may take 2-5 minutes)");
     }
   }
   
   if (rtcmStatus == ePPL_Success && len > 0) {
     totalRTCMBytes += len;
     rtcmMessageCount++;
     
     static uint32_t lastNewDataLog = 0;
     if (millis() - lastNewDataLog > 15000) {  // Reduced frequency from 5s to 15s for performance
       lastNewDataLog = millis();
       DBG("🎉 PPL CONVERGED! Generated RTK corrections: " + String(len) + " bytes");
     }
     
     // Cache RTCM data for 1Hz NTRIP transmission (instead of immediate transmission)
     if (len <= PPL_MAX_RTCM_BUFFER) {
       memcpy(cachedRTCMBuffer, rtcmBuffer, len);
       cachedRTCMLength = len;
       lastRTCMCache = millis();
       
       static uint32_t cacheCount = 0;
       cacheCount++;
       if (cacheCount % 10 == 0) {  // Log every 10th cache operation
         DBG("RTCM cached for 1Hz NTRIP transmission: " + String(len) + " bytes (cache #" + String(cacheCount) + ")");
       }
     } else {
       DBG("ERROR: RTCM data too large for cache: " + String(len) + " bytes (max: " + String(PPL_MAX_RTCM_BUFFER) + ")");
     }
   }
 }
 
 /* ---------- NTRIP Server Functions ----------------------------- */
 
 // Simple RTCM message type parser for sourcetable
 String parseRTCMTypes(uint8_t *data, size_t len) {
   String types = "";
   static uint16_t msgLen = 0, msgType = 0;
   static uint8_t state = 0;
   
   for (size_t i = 0; i < len; i++) {
     switch (state) {
       case 0: if (data[i] == 0xD3) state = 1; break;
       case 1: msgLen = ((data[i] & 3) << 8); state = 2; break;
       case 2: msgLen |= data[i]; msgLen += 3; state = 3; break;
       case 3: msgType = ((uint16_t)data[i]) << 4; state = 4; msgLen--; break; 
       case 4: msgType |= (data[i] >> 4); types += String(msgType) + " "; state = 5; msgLen--; break;
       case 5: if (--msgLen == 0) state = 0; break;
     }
   }
   return types;
 }
 
 void handleNTRIP() {
   // Accept new NTRIP clients
   if (!ntripClient || !ntripClient.connected()) {
     ntripClient.stop();
     WiFiClient c = ntripServer.available();
     if (c) {
       DBG("🔗 === NEW NTRIP CONNECTION ===");
       DBG("📍 Client IP: " + c.remoteIP().toString() + ":" + String(c.remotePort()));
       DBG("🌐 Local Server: " + WiFi.localIP().toString() + ":2101");
       
       // Read complete request with detailed logging
       String request = "";
       unsigned long timeout = millis() + 3000;
       bool requestComplete = false;
       int lineCount = 0;
       
       while (c.connected() && millis() < timeout && !requestComplete) {
         if (c.available()) {
           String line = c.readStringUntil('\n');
           line.trim();
           request += line + "\n";
           lineCount++;
           
           if (lineCount == 1) {
             DBG("📥 NTRIP Request Line: " + line);
           }
           
           if (line.length() == 0) {
             requestComplete = true;
             DBG("✅ Request complete (" + String(lineCount) + " lines received)");
           }
         }
       }
       
       if (!requestComplete) {
         DBG("⚠️ Request timeout after 3 seconds");
       }
       
       // Handle sourcetable request
       if (request.indexOf("GET / ") >= 0 || request.indexOf("GET /SOURCETABLE") >= 0) {
         DBG("📋 SOURCETABLE REQUEST - Sending mountpoint information");
         
         String sourceLine = "STR;rtcm;ESP32-PPL;RTCM 3.3;1005(10),1019(30),1020(30),1042(30),1046(30);2;GPS+GLO+GAL+BDS;ESP32;USA;";
         if (baseStationSet && rtcm1005Data.valid) {
           sourceLine += String(rtcm1005Data.latitude, 6) + ";" + String(rtcm1005Data.longitude, 6) + ";1;1;PPL-Base;none;N;N;2400;PointPerfect\r\n";
           DBG("📍 Base station coordinates included: " + String(rtcm1005Data.latitude, 6) + "°, " + String(rtcm1005Data.longitude, 6) + "°");
         } else {
           sourceLine += "0.000000;0.000000;1;1;PPL-Base;none;N;N;2400;PointPerfect-NoFix\r\n";
           DBG("⚠️ No base station coordinates available yet");
         }
         
         String sourceTable = "SOURCETABLE 200 OK\r\n" + sourceLine + "ENDSOURCETABLE\r\n";
         
         c.print(sourceTable);
         c.flush();
         c.stop();
         DBG("✅ Sourcetable sent successfully (" + String(sourceTable.length()) + " bytes)");
         return;
       }
       
       // Handle mountpoint request
       if (request.indexOf("GET /rtcm ") >= 0) {
         DBG("🎯 RTCM STREAM REQUEST - Setting up data stream");
         c.print("ICY 200 OK\r\n");
         c.print("Ntrip-Version: Ntrip/1.0\r\n");
         c.print("Server: ESP32-PPL/1.0\r\n");
         c.print("Connection: close\r\n");
         c.print("Content-Type: gnss/data\r\n");
         c.print("\r\n");
         c.flush();
         
         ntripClient = c;
         DBG("✅ NTRIP client connected for RTK corrections streaming");
         DBG("🚀 Ready to serve cached RTCM data at 1Hz");
         
         // Reset transmission counter for this new client
         ntripTransmissionCount = 0;
       } else {
         DBG("❌ Unknown NTRIP request - closing connection");
         c.stop();
       }
     }
   }
   
   // Handle client disconnection with detailed logging
   static bool wasConnected = false;
   bool isConnected = (ntripClient && ntripClient.connected());
   
   if (wasConnected && !isConnected) {
     DBG("❌ === NTRIP CLIENT DISCONNECTED ===");
     DBG("📊 Session stats: " + String(ntripTransmissionCount) + " transmissions sent");
     DBG("📈 Total data served: " + String(ntripTransmissionCount * cachedRTCMLength) + " bytes");
     ntripClient.stop();
   }
   wasConnected = isConnected;
   
   // Send cached RTCM at 1Hz to connected NTRIP client
   if (ntripClient && ntripClient.connected() && cachedRTCMLength > 0) {
     if (millis() - lastNTRIPTransmission >= 1000) { // 1Hz transmission
       lastNTRIPTransmission = millis();
       ntripTransmissionCount++;
       
       size_t bytesWritten = ntripClient.write(cachedRTCMBuffer, cachedRTCMLength);
       ntripClient.flush();
       
       // Log transmission details periodically
       if (ntripTransmissionCount % 10 == 0) {  // Every 10 seconds
         DBG("📡 NTRIP 1Hz: Sent " + String(bytesWritten) + "/" + String(cachedRTCMLength) + 
             " bytes to " + ntripClient.remoteIP().toString() + " (transmission #" + String(ntripTransmissionCount) + ")");
         
         if (bytesWritten != cachedRTCMLength) {
           DBG("⚠️ WARNING: Incomplete transmission detected!");
         }
       }
       
       // Connection health check
       if (ntripTransmissionCount % 30 == 0) {  // Every 30 seconds
         DBG("💓 NTRIP Health: Client " + ntripClient.remoteIP().toString() + " - " + 
             String(ntripTransmissionCount) + " transmissions, " + 
             String(ntripTransmissionCount * cachedRTCMLength) + " bytes total");
       }
     }
   } else if (ntripClient && ntripClient.connected() && cachedRTCMLength == 0) {
     // Client connected but no RTCM data available
     static uint32_t lastNoDataWarning = 0;
     if (millis() - lastNoDataWarning > 30000) {  // Every 30 seconds
       lastNoDataWarning = millis();
       DBG("⏳ NTRIP: Client connected but no RTCM data cached yet (PPL still converging)");
     }
   }
 }
 
 /* ---------- Web Debug Interface -------------------------------- */
 
 void handleWebDebug() {
   WiFiClient webClient = webServer.available();
   if (webClient) {
     webClient.setTimeout(1000);
     String request = webClient.readStringUntil('\r');
     webClient.flush();
     
     // Send HTTP response
     webClient.println("HTTP/1.1 200 OK");
     webClient.println("Content-Type: text/html");
     webClient.println("Connection: close");
     webClient.println();
     
     // HTML page
     webClient.println("<!DOCTYPE html><html><head>");
     webClient.println("<title>PPL RTK Base Station</title>");
     webClient.println("<meta http-equiv='refresh' content='10'>");
     webClient.println("<style>body{font-family:Arial;margin:20px;} .good{color:green;} .warn{color:orange;} .bad{color:red;}</style>");
     webClient.println("</head><body>");
     
     webClient.println("<h1>PPL RTK Base Station Status</h1>");
     webClient.println("<p><strong>IP:</strong> " + WiFi.localIP().toString() + " | ");
     webClient.println("<strong>Runtime:</strong> " + String(millis()/1000) + "s | ");
     webClient.println("<strong>Free Heap:</strong> " + String(ESP.getFreeHeap()) + " bytes</p>");
     
     // Data flow status
     webClient.println("<h2>Data Flow Status</h2>");
     webClient.println("<table border='1'>");
     webClient.println("<tr><th>Source</th><th>Message</th><th>Count</th><th>Status</th></tr>");
     
     webClient.println("<tr><td rowspan='5'>PX1105R</td><td>RTCM 1019 (GPS)</td><td>" + String(cntRTCM1019) + "</td><td class='" + 
                       String(cntRTCM1019 > 0 ? "good'>✓" : "bad'>✗") + "</td></tr>");
     webClient.println("<tr><td>RTCM 1020 (GLONASS)</td><td>" + String(cntRTCM1020) + "</td><td class='" + 
                       String(cntRTCM1020 > 0 ? "good'>✓" : "bad'>✗") + "</td></tr>");
     webClient.println("<tr><td>RTCM 1042 (BeiDou)</td><td>" + String(cntRTCM1042) + "</td><td class='" + 
                       String(cntRTCM1042 > 0 ? "good'>✓" : "bad'>✗") + "</td></tr>");
     webClient.println("<tr><td>RTCM 1046 (Galileo)</td><td>" + String(cntRTCM1046) + "</td><td class='" + 
                       String(cntRTCM1046 > 0 ? "good'>✓" : "bad'>✗") + "</td></tr>");
     webClient.println("<tr><td>RTCM 1005 (Position)</td><td>" + String(cntRTCM1005) + "</td><td class='" + 
                       String(cntRTCM1005 > 0 ? "good'>✓" : "bad'>✗") + "</td></tr>");
     
     webClient.println("<tr><td rowspan='2'>NEO-D9S</td><td>PMP Messages</td><td>" + String(cntPMP) + "</td><td class='" + 
                       String(cntPMP > 0 ? "good'>✓" : "bad'>✗") + "</td></tr>");
     webClient.println("<tr><td>SPARTN Data</td><td>" + String(cntSPARTN) + "</td><td class='" + 
                       String(cntSPARTN > 0 ? "good'>✓" : "bad'>✗") + "</td></tr>");
     
     webClient.println("<tr><td rowspan='2'>Generated</td><td>NMEA GGA</td><td>" + String(cntGGA) + "</td><td class='" + 
                       String(cntGGA > 0 ? "good'>✓" : "bad'>✗") + "</td></tr>");
     webClient.println("<tr><td>NMEA ZDA</td><td>" + String(cntZDA) + "</td><td class='" + 
                       String(cntZDA > 0 ? "good'>✓" : "bad'>✗") + "</td></tr>");
     
     webClient.println("<tr><td>PPL</td><td>RTK Corrections</td><td>" + String(rtcmMessageCount) + "</td><td class='" + 
                       String(rtcmMessageCount > 0 ? "good'>✓ ACTIVE" : "warn'>⏳ CONVERGING") + "</td></tr>");
     webClient.println("</table>");
     
     // Base station info
     webClient.println("<h2>Base Station</h2>");
     if (baseStationSet && rtcm1005Data.valid) {
       webClient.println("<p class='good'>Status: ✓ COORDINATES SET</p>");
       webClient.println("<p><strong>Latitude:</strong> " + String(rtcm1005Data.latitude, 8) + "°<br>");
       webClient.println("<strong>Longitude:</strong> " + String(rtcm1005Data.longitude, 8) + "°<br>");
       webClient.println("<strong>Height:</strong> " + String(rtcm1005Data.height, 3) + " m<br>");
       webClient.println("<strong>Station ID:</strong> " + String(rtcm1005Data.stationId) + "</p>");
     } else {
       webClient.println("<p class='warn'>Status: ⏳ WAITING FOR RTCM 1005</p>");
     }
     
     // GPS time info
     webClient.println("<h2>GPS Time</h2>");
     if (gpsTime.valid) {
       uint32_t totalSeconds = gpsTime.timeOfWeek / 1000;
       uint32_t hours = (totalSeconds / 3600) % 24;
       uint32_t minutes = (totalSeconds / 60) % 60;
       uint32_t seconds = totalSeconds % 60;
       
       webClient.println("<p class='good'>Status: ✓ TIME SYNCHRONIZED</p>");
       webClient.println("<p><strong>GPS Week:</strong> " + String(gpsTime.weekNumber) + "<br>");
       webClient.println("<strong>Time of Week:</strong> " + String(gpsTime.timeOfWeek) + " ms<br>");
       webClient.println("<strong>UTC Time:</strong> " + String(hours) + ":" + String(minutes) + ":" + String(seconds) + "<br>");
       webClient.println("<strong>Leap Seconds:</strong> " + String(gpsTime.currentLeapSec) + "</p>");
     } else {
       webClient.println("<p class='warn'>Status: ⏳ WAITING FOR GPS TIME</p>");
     }
     
     // NTRIP status
     webClient.println("<h2>NTRIP Server</h2>");
     webClient.println("<p><strong>Port:</strong> 2101<br>");
     webClient.println("<strong>Mountpoint:</strong> /rtcm<br>");
     if (ntripClient && ntripClient.connected()) {
       webClient.println("<strong>Client:</strong> <span class='good'>✓ CONNECTED (" + ntripClient.remoteIP().toString() + ")</span><br>");
     } else {
       webClient.println("<strong>Client:</strong> <span class='warn'>⏳ WAITING FOR CONNECTION</span><br>");
     }
     webClient.println("<strong>Total RTK Data:</strong> " + String(totalRTCMBytes) + " bytes<br>");
     webClient.println("<strong>RTK Messages:</strong> " + String(rtcmMessageCount) + "</p>");
     
     // PPL status
     webClient.println("<h2>PPL Status</h2>");
     bool pplReady = (cntSPARTN > 0 && cntRTCM1019 > 0 && cntGGA > 0 && cntZDA > 0);
     if (rtcmMessageCount > 0) {
       webClient.println("<p class='good'>Status: ✓ GENERATING RTK CORRECTIONS</p>");
     } else if (pplReady) {
       webClient.println("<p class='warn'>Status: ⏳ CONVERGENCE IN PROGRESS</p>");
       webClient.println("<p>All required data streams are active. PPL typically takes 2-5 minutes to converge.</p>");
     } else {
       webClient.println("<p class='bad'>Status: ✗ WAITING FOR DATA</p>");
       webClient.println("<p>Missing: ");
       if (cntSPARTN == 0) webClient.println("SPARTN ");
       if (cntRTCM1019 == 0) webClient.println("RTCM-1019 ");
       if (cntGGA == 0) webClient.println("GGA ");
       if (cntZDA == 0) webClient.println("ZDA ");
       webClient.println("</p>");
     }
     
     // Current NMEA
     webClient.println("<h2>Current NMEA</h2>");
     webClient.println("<p><strong>GGA:</strong> " + currentGGA + "<br>");
     webClient.println("<strong>ZDA:</strong> " + currentZDA + "</p>");
     
     // Connection instructions
     webClient.println("<h2>Usage</h2>");
     webClient.println("<p><strong>NTRIP Connection:</strong><br>");
     webClient.println("Host: " + WiFi.localIP().toString() + "<br>");
     webClient.println("Port: 2101<br>");
     webClient.println("Mountpoint: rtcm<br>");
     webClient.println("Username/Password: any</p>");
     
     webClient.println("</body></html>");
     webClient.flush();
     webClient.stop();
   }
 }
 
 /* ---------- Main Data Processing Functions ------------------ */
 
 // Parse PX1105R data (RTCM + GPS Time)
 void parsePXData() {
   // Query GPS time periodically
   if (millis() - lastGpsTimeQuery >= gpsTimeQueryInterval) {
     queryGpsTime();
     lastGpsTimeQuery = millis();
   }
   
   // Read incoming data
   while (pxSerial.available() && pxBufferIndex < PX_BUFFER_SIZE - 1) {
     pxBuffer[pxBufferIndex++] = pxSerial.read();
   }
   
   // Process buffer for RTCM and PX1105R binary messages
   for (int i = 0; i <= pxBufferIndex - 3; i++) {
     // Look for RTCM preamble (0xD3)
     if (pxBuffer[i] == 0xD3) {
       if (i + 3 <= pxBufferIndex) {
         uint16_t header = (pxBuffer[i+1] << 8) | pxBuffer[i+2];
         uint16_t length = header & 0x3FF;
         uint16_t messageType = (pxBuffer[i+3] << 4) | (pxBuffer[i+4] >> 4);
         
         // Check if we have complete message
         if (i + 3 + length + 3 <= pxBufferIndex) {
           countRTCMMessage(messageType, &pxBuffer[i], 3 + length + 3);
           
           // Remove processed data
           int msgSize = 3 + length + 3;
           memmove(&pxBuffer[i], &pxBuffer[i + msgSize], pxBufferIndex - i - msgSize);
           pxBufferIndex -= msgSize;
           i--;
         }
       }
     }
     // Look for PX1105R binary response (0xA0, 0xA1)
     else if (pxBuffer[i] == 0xA0 && i + 1 < pxBufferIndex && pxBuffer[i+1] == 0xA1) {
       if (i + 4 <= pxBufferIndex) {
         uint16_t payloadLength = (pxBuffer[i+2] << 8) | pxBuffer[i+3];
         
         if (i + 4 + payloadLength + 3 <= pxBufferIndex) {
           // Check for GPS time response (0x64, 0x8E)
           if (payloadLength >= 2 && pxBuffer[i+4] == 0x64 && pxBuffer[i+5] == 0x8E) {
             if (payloadLength == 15) {
               parseGpsTimeResponse(&pxBuffer[i+4]);
             } else {
               DBG("GPS time response wrong length: " + String(payloadLength) + " (expected 15)");
             }
           }
           // Debug other binary messages occasionally
           else if (payloadLength >= 2) {
             static uint32_t lastBinaryLog = 0;
             if (millis() - lastBinaryLog > 10000) {
               lastBinaryLog = millis();
               DBG("PX Binary: 0x" + String(pxBuffer[i+4], HEX) + " 0x" + String(pxBuffer[i+5], HEX) + " len=" + String(payloadLength));
             }
           }
           
           // Remove processed data
           int msgSize = 4 + payloadLength + 3;
           memmove(&pxBuffer[i], &pxBuffer[i + msgSize], pxBufferIndex - i - msgSize);
           pxBufferIndex -= msgSize;
           i--;
         }
       }
     }
   }
   
   // Prevent buffer overflow
   if (pxBufferIndex > PX_BUFFER_SIZE - 100) {
     memmove(pxBuffer, &pxBuffer[PX_BUFFER_SIZE/2], PX_BUFFER_SIZE/2);
     pxBufferIndex = PX_BUFFER_SIZE/2;
   }
 }
 
 // Parse D9S data using SparkFun library - much simpler!
 void parseD9SData() {
   if (!d9sConnected) return;
   
   // SparkFun library handles all UBX parsing automatically
   myD9S.checkUblox();      // Check for new data and process it
   myD9S.checkCallbacks();  // Process any pending callbacks (like PMP messages)
 }
 
 /* ---------- Main Setup ------------------------------------------ */
 void setup() {
   Serial.begin(115200);
   delay(500);
   
   Serial.println("\n=== PPL RTK Base Station ===");
   Serial.println("PX1105R (RTCM) + NEO-D9S (SPARTN) + PPL");
   Serial.println(PPL_SDK_VERSION);
   
   // Initialize WiFi
   WiFi.begin(WIFI_SSID, WIFI_PASS);
   DBG("Wi-Fi connecting...");
   while (WiFi.status() != WL_CONNECTED) { 
     delay(100); 
     Serial.print('.'); 
   }
   DBG("Wi-Fi connected – IP: " + WiFi.localIP().toString());
   
   if (ENABLE_NTRIP) { 
     ntripServer.begin(); 
     DBG("NTRIP caster on :2101"); 
   }
   
   // Web debug disabled for performance
   // if (ENABLE_WEB_DEBUG) { 
   //   webServer.begin(); 
   //   DBG("Web debug server on :80"); 
   //   DBG("Visit http://" + WiFi.localIP().toString() + " for status");
   // }
   
   // Initialize hardware modules
   setupPX1105R();
   setupD9S(); 
   setupPPL();
   
   DBG("Setup complete - starting RTK base station...");
 }
 
 /* ---------- Main Loop ------------------------------------------- */
 void loop() {
   // Parse data from both modules
   parsePXData();
   parseD9SData();
   
   // Generate NMEA from RTCM 1005 + GPS time
   generateNMEAFromRTCM();
   
   // Handle PPL RTCM output (only when new data was sent)
   checkPPLOutput();
   
   // Handle NTRIP connections
   if (ENABLE_NTRIP) {
     handleNTRIP();
   }
   
   // Web debug disabled for performance
   // if (ENABLE_WEB_DEBUG) {
   //   handleWebDebug();
   // }
   
   // Print periodic status with satellite details (reduced frequency for performance)
   static uint32_t lastStatus = 0;
   if (millis() - lastStatus > 10000) {  // Every 10 seconds instead of 5
     lastStatus = millis();
     
     // Build satellite constellation status
     String satStatus = "Satellites - ";
     if (activeSats.gps_count > 0) {
       satStatus += "GPS:" + String(activeSats.gps_count) + " (";
       for (int i = 0; i < activeSats.gps_count; i++) {
         satStatus += String(activeSats.gps_sats[i]);
         if (i < activeSats.gps_count - 1) satStatus += ",";
       }
       satStatus += ") ";
     }
     
     if (activeSats.glo_count > 0) {
       satStatus += "GLO:" + String(activeSats.glo_count) + " (";
       for (int i = 0; i < activeSats.glo_count; i++) {
         satStatus += String(activeSats.glo_sats[i]);
         if (i < activeSats.glo_count - 1) satStatus += ",";
       }
       satStatus += ") ";
     }
     
     if (activeSats.bds_count > 0) {
       satStatus += "BDS:" + String(activeSats.bds_count) + " (";
       for (int i = 0; i < activeSats.bds_count; i++) {
         satStatus += String(activeSats.bds_sats[i]);
         if (i < activeSats.bds_count - 1) satStatus += ",";
       }
       satStatus += ") ";
     }
     
     if (activeSats.gal_count > 0) {
       satStatus += "GAL:" + String(activeSats.gal_count) + " (";
       for (int i = 0; i < activeSats.gal_count; i++) {
         satStatus += String(activeSats.gal_sats[i]);
         if (i < activeSats.gal_count - 1) satStatus += ",";
       }
       satStatus += ") ";
     }
     
     if (activeSats.gps_count == 0 && activeSats.glo_count == 0 && 
         activeSats.bds_count == 0 && activeSats.gal_count == 0) {
       satStatus += "None detected";
     }
     
     DBG(satStatus);
     
     // Message counts summary
     DBG("Messages - 1019:" + String(cntRTCM1019) + " 1020:" + String(cntRTCM1020) + 
         " 1042:" + String(cntRTCM1042) + " 1046:" + String(cntRTCM1046) + " 1005:" + String(cntRTCM1005) +
         " | D9S: PMP:" + String(cntPMP) + " SPARTN:" + String(cntSPARTN) + 
         " | PPL: RTCM:" + String(rtcmMessageCount));
     
     // NTRIP status summary
     if (ntripClient && ntripClient.connected()) {
       DBG("📡 NTRIP: Client " + ntripClient.remoteIP().toString() + " connected - " + 
           String(ntripTransmissionCount) + " transmissions, " + 
           String(cachedRTCMLength) + " bytes cached");
     } else {
       DBG("📡 NTRIP: No client connected (cached: " + String(cachedRTCMLength) + " bytes)");
     }
     
     // Reset satellite lists every 2 minutes to show only currently active satellites
     static uint32_t lastSatReset = 0;
     if (millis() - lastSatReset > 120000) { // 2 minutes
       lastSatReset = millis();
       activeSats.gps_count = 0;
       activeSats.glo_count = 0;
       activeSats.bds_count = 0;
       activeSats.gal_count = 0;
       DBG("Satellite list reset - showing new ephemeris updates");
     }
     
     // PPL convergence status
     if (rtcmMessageCount == 0) {
       bool hasInputs = (cntSPARTN > 0 && cntRTCM1019 > 0 && cntGGA > 0 && cntZDA > 0);
       if (hasInputs) {
         DBG("PPL Status: All data streams active - convergence in progress (runtime: " + String(millis()/1000) + "s)");
       } else {
         String missing = "";
         if (cntSPARTN == 0) missing += "SPARTN ";
         if (cntRTCM1019 == 0) missing += "RTCM-1019 ";
         if (cntGGA == 0) missing += "GGA ";
         if (cntZDA == 0) missing += "ZDA ";
         DBG("PPL Status: Missing data - " + missing);
       }
     }
   }
 }
 