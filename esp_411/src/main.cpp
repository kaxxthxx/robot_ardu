// #include <Arduino.h>

// #include "BluetoothSerial.h"

// const char *pin = "1234"; 

// String device_name = "ESP32-BT-Slave";

// #if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
// #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
// #endif

// #if !defined(CONFIG_BT_SPP_ENABLED)
// #error Serial Bluetooth not available or not enabled. It is only available ...
// #endif

// BluetoothSerial SerialBT;

// void setup() {
// Serial.begin(115200);
// SerialBT.begin(device_name); 
// Serial.printf("The device \"%s\" is started...\n", device_name.c_str());

// #ifdef USE_PIN
// SerialBT.setPin(pin);
// Serial.println("Using PIN");
// #endif
// }

// void loop() {
// if (Serial.available()) {
// SerialBT.write(Serial.read());
// }
// if (SerialBT.available()) {
// Serial.write(SerialBT.read());
// }
// delay(20);
// }

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// UUIDs for service and characteristic
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define LED_PIN 4  // D4번 핀 (GPIO 4)

BLECharacteristic *pCharacteristicTX;
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("iPhone connected");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("iPhone disconnected");
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0) {
      Serial.print("Received from iPhone: ");
      Serial.println(value.c_str());

      // LED 제어
      if (value == "on") {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("LED ON");
      } else if (value == "off") {
        digitalWrite(LED_PIN, LOW);
        Serial.println("LED OFF");
      } else {
        Serial.println("Unknown command");
      }

      // Echo back to iPhone
      pCharacteristicTX->setValue(value);
      pCharacteristicTX->notify();
    }
  }
};

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // 초기 LED OFF

  BLEDevice::init("ESP32-BLE-UART");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristicRX = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID_RX,
                                          BLECharacteristic::PROPERTY_WRITE
                                        );
  pCharacteristicRX->setCallbacks(new MyCallbacks());

  pCharacteristicTX = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("Waiting for iPhone to connect...");
}

void loop() {
  // 아무것도 안함
}
