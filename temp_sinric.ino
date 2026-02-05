#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <WiFi.h>
#include "SinricPro.h"
#include "SinricProTemperaturesensor.h"

// ===== CONFIG =====
const char* ssid = "";
const char* password = "";

#define APP_KEY    ""
#define APP_SECRET ""
#define TEMP_ID    ""

#define RAPT_MANUFACTURER_ID 0x4152
#define SCAN_TIME_SEC 10 

// ===== GLOBALS =====
float lastTemp = NAN;
bool foundData = false;

// ===== DEBUG FUNCTION =====
void printFreeMemory(const char* label) {
  Serial.printf("[%s] Free Heap: %u bytes\n", label, ESP.getFreeHeap());
}

// ===== BLE CALLBACK =====
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (!advertisedDevice.haveManufacturerData()) return;

    uint8_t* payload = advertisedDevice.getPayload();
    size_t len = advertisedDevice.getPayloadLength();

    for (int i = 0; i < len; i++) {
      uint8_t blockLen = payload[i];
      if (i + 1 < len && payload[i + 1] == 0xFF) {
        uint16_t mId = (payload[i + 3] << 8) | payload[i + 2];
        if (mId == RAPT_MANUFACTURER_ID) {
          // Temperature Extraction
          uint16_t tRaw = (payload[i + 13] << 8) | payload[i + 14];
          lastTemp = (float)tRaw / 128.0 - 273.15;
          foundData = true;
          return;
        }
      }
      i += blockLen;
    }
  }
};

// ===== PHASES =====

void runBleScanPhase() {
  Serial.println("\n>>> PHASE 1: BLE SCAN");
  printFreeMemory("Pre-BLE");
  
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
  pBLEScan->setActiveScan(true);
  
  Serial.print("Scanning for RAPT Pill...");
  pBLEScan->start(SCAN_TIME_SEC, false);
  
  // --- STRICT BLE CLEANUP ---
  pBLEScan->stop();
  pBLEScan->clearResults(); 
  BLEDevice::deinit(true);  
  
  delay(500); 
  printFreeMemory("Post-BLE");
}

void runCloudPhase() {
  Serial.println("\n>>> PHASE 2: WIFI & SINRIC PRO");
  printFreeMemory("Pre-WiFi");
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
    printFreeMemory("Post-WiFi");

    // Setup SinricPro
    SinricProTemperaturesensor &mySensor = SinricPro[TEMP_ID];
    
    // --- CORRECTED CLEANUP FOR NEW SINRICPRO ---
    // Removed direct call to disconnect() as it is private
    SinricPro.begin(APP_KEY, APP_SECRET);

    // Give it a few seconds to establish WebSocket connection
    unsigned long startSocket = millis();
    while (millis() - startSocket < 5000) {
      SinricPro.handle(); 
      delay(10);
    }

    if (foundData) {
      Serial.printf("Reporting Temp: %.2f C\n", lastTemp);
      mySensor.sendTemperatureEvent(lastTemp, 0.0);
    } else {
      Serial.println("No BLE data to report this cycle.");
    }

    // Allow time for the outgoing packet to clear the buffer
    for(int i=0; i<100; i++) { SinricPro.handle(); delay(10); }
    Serial.println("Transmission finished.");

  } else {
    Serial.println("\nWiFi Failed.");
  }
  
  // Clean up WiFi before reboot
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);
}

// ===== CORE =====

void setup() {
  Serial.begin(115200);
  while(!Serial); 
  delay(2000);
  
  Serial.println("\n\n============================");
  Serial.println("XIAO C3 RAPT -> BEERvBRIDGE");
  Serial.println("============================");

  runBleScanPhase();
  
  if (foundData) {
     runCloudPhase();
  } else {
     Serial.println("Pill not found. Skipping WiFi phase.");
  }

  Serial.println("Cycle Complete. Rebooting in 1s...");
  delay(1000);
  ESP.restart();
}

void loop() {}
