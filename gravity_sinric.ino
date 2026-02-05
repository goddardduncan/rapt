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

// YOUR SINRIC PRO CREDENTIALS
#define APP_KEY    ""
#define APP_SECRET ""
#define TEMP_ID    ""

#define RAPT_MANUFACTURER_ID 0x4152
#define SCAN_TIME_SEC 10 // Scanning duration

// ===== GRAVITY SETTINGS =====
const float ORIGINAL_GRAVITY = 1.050; 
const float FINAL_GRAVITY = 1.010;

// ===== GLOBALS =====
float lastGravity = NAN; // SG value
bool foundData = false;

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
          // Gravity (SG) Extraction
          uint8_t gArr[4] = {payload[i + 18], payload[i + 17], payload[i + 16], payload[i + 15]};
          memcpy(&lastGravity, gArr, 4);
          lastGravity /= 1000.0;
          
          foundData = true;
          return;
        }
      }
      i += blockLen;
    }
  }
};

// ===== LOGIC: SG to Completion % (Mapped to 0-100) =====
float calculateCompletionPercentage(float currentSG, float og, float fg) {
  if (currentSG >= og) return 0.0;
  if (currentSG <= fg) return 100.0;
  
  // Calculate completion percentage: ((OG - Current) / (OG - FG)) * 100
  return ((og - currentSG) / (og - fg)) * 100.0;
}

// ===== PHASES =====

void runBleScanPhase() {
  Serial.println("\n>>> PHASE 1: BLE SCAN");
  
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
  pBLEScan->setActiveScan(true);
  
  Serial.print("Scanning for RAPT Pill...");
  pBLEScan->start(SCAN_TIME_SEC, false);
  
  pBLEScan->stop();
  pBLEScan->clearResults(); 
  BLEDevice::deinit(true);  
  
  delay(500); 
}

void runCloudPhase() {
  Serial.println("\n>>> PHASE 2: WIFI & SINRIC PRO");
  
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

    // Setup SinricPro
    SinricProTemperaturesensor &mySensor = SinricPro[TEMP_ID];
    SinricPro.begin(APP_KEY, APP_SECRET);

    unsigned long startSocket = millis();
    while (millis() - startSocket < 5000) {
      SinricPro.handle(); 
      delay(10);
    }

    if (foundData) {
      float completionPercent = calculateCompletionPercentage(lastGravity, ORIGINAL_GRAVITY, FINAL_GRAVITY);
      Serial.printf("Reporting -> SG: %.4f | Completion: %.1f%% (Reported as Temp)\n", 
                    lastGravity, completionPercent);
      
      // sendTemperatureEvent(Temperature, Humidity)
      // Mapping calculated completion percentage to the "Temperature" field in Sinric
      // Humidity is sent as 0.0 because we are not using it.
      mySensor.sendTemperatureEvent(completionPercent, 0.0);
    }

    for(int i=0; i<100; i++) { SinricPro.handle(); delay(10); }
    Serial.println("Transmission finished.");
  } else {
    Serial.println("\nWiFi Failed.");
  }
  
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
  Serial.println("XIAO C3 RAPT -> BEERBRIDGE");
  Serial.println("============================");

  // Run Phase 1: BLE
  runBleScanPhase();
  
  // Run Phase 2: Cloud (only if data found)
  if (foundData) {
     runCloudPhase();
  } else {
     Serial.println("Pill not found. Skipping WiFi phase.");
  }

  Serial.println("Cycle Complete. Rebooting in 1s...");
  delay(1000);
  ESP.restart();
}

void loop() {
}
