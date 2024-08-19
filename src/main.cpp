#include <Arduino.h>
#include "BLEDevice.h"

#define bleServerName "MANUEL_ESP32"
#define LED_PIN 16  
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"
#define CHARACTERISTIC_UUID "ec0e8b64-33ae-11ec-8d3d-0242ac130003" // Unique ID for the characteristic

static BLEUUID potServiceUUID(SERVICE_UUID);
static BLEUUID pwmCharacteristicUUID(CHARACTERISTIC_UUID);

static boolean doConnect = false;
static boolean connected = false;

static BLEAddress *pServerAddress;
BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;

const int pwmChannel = 0;
const int freq = 5000;        // 5 kHz
const int resolution = 8;     // 8 bits (0-255)

bool connectToServer(BLEAddress pAddress) {
  Serial.println("Attempting to connect to server...");
  pClient = BLEDevice::createClient();

  if (!pClient->connect(pAddress)) {
    Serial.println("Failed to connect to server");
    return false;
  }

  Serial.println(" - Connected to server");

  BLERemoteService* pRemoteService = pClient->getService(potServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our client service UUID: ");
    Serial.println(potServiceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }

  pRemoteCharacteristic = pRemoteService->getCharacteristic(pwmCharacteristicUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our PWM characteristic UUID: ");
    Serial.println(pwmCharacteristicUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }

  Serial.println("Found characteristic UUID!");

  // Send MAC address to the server
  std::string macAddress = BLEDevice::getAddress().toString();
  pRemoteCharacteristic->writeValue(macAddress);

  pRemoteCharacteristic->registerForNotify([](BLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    uint32_t pwmValue = pData[0];  // Assuming the value is sent as a single byte

    ledcWrite(pwmChannel, pwmValue); 

    Serial.print("Received PWM Value: ");
    Serial.println(pwmValue);
  });

  return true;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == bleServerName) {
      advertisedDevice.getScan()->stop();
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      doConnect = true;
      Serial.println("Device found. Connecting!");

      // int rssi = advertisedDevice.getRSSI();
      // Serial.print("RSSI: ");
      // Serial.println(rssi);
    }
  }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application");

  BLEDevice::init("");

  ledcSetup(pwmChannel, freq, resolution); 
  ledcAttachPin(LED_PIN, pwmChannel);    

  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
}

void loop() {
  if (doConnect) {
    if (connectToServer(*pServerAddress)) {
      while (doConnect) {
        int rssi = pClient->getRssi();
        Serial.print("RSSI: ");
        Serial.println(rssi);
        delay(1000);
      }
      Serial.println("We are now connected to the BLE Server");
      connected = true;
    } else {
      Serial.println("We have failed to connect to the server; Restart your device to rescan");
    }
    doConnect = false;
  }
  delay(1000);
}