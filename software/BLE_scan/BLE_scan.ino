/*
   Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

int scanTime = 10; //In seconds
BLEScan* pBLEScan;

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.printf("Advertised Device: %s", advertisedDevice.toString().c_str());
      Serial.print(" RSSI:");
      Serial.print(advertisedDevice.getRSSI());
      if(advertisedDevice.haveServiceUUID())
      {
        Serial.print(" UUID:");
        Serial.print(advertisedDevice.getServiceUUID().toString().c_str());
        
      }

      //Serial.printf("Address: %s \n", advertisedDevice.getAddress().toString().c_str());
      uint8_t* data =    advertisedDevice.getPayload();
      Serial.printf("Payload:");
      for(int i = 0; i < sizeof(data) / sizeof(data[0]); ++i){
          Serial.printf("%d ", data[i]);
      }
      Serial.printf("\n");
    }
};

void setup() {
  Serial.begin(9600);
  Serial.println("Scanning...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
}

void loop() {
  BLEScanResults foundDevices = pBLEScan->start(scanTime);
  Serial.print("Devices found: ");
  Serial.println(foundDevices.getCount());
  Serial.println("Scan done!");
  delay(2000);
}
