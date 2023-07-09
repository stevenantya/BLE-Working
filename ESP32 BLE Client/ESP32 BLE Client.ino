/**
 * A BLE client example that is rich in capabilities.
 * There is a lot new capabilities implemented.
 * author unknown
 * updated by chegewara
 */

#include "BLEDevice.h"
#include <WiFi.h>
#include <PubSubClient.h>

//#include "BLEScan.h"

int ledPin = 2;
int ledRed = 17;
int ledYellow = 25;
int ledGreen = 26;

// Replace these with your WiFi and MQTT broker details
const char* ssid = "myuan";
const char* password = "98765432";
const char* mqttServer = "192.168.194.135";
const int mqttPort = 1883;
const char* mqttUser = "pi";  // If needed
const char* mqttPassword = "123456"; // If needed
const char* mqttTopic = "test/data";

WiFiClient espClient;
PubSubClient client(espClient);

// The remote service we wish to connect to.
static BLEUUID accelerometerSensorServiceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// The characteristic of the remote service we are interested in.
static BLEUUID acclerometerCharacteristicUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");



static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristicX;
static BLEAdvertisedDevice* myDevice;

void setupWiFi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
    // if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

String messageAppended = "";
int counter = 1;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    // Serial.print(millis());
    // Serial.print("  ");
    // Serial.print("Notify callback for characteristic ");
    // String identifier = pBLERemoteCharacteristic->getUUID().toString().c_str();
    // if (identifier.equals("beb5483e-36e1-4688-b7f5-ea07361b26a8")) {
    //   // Serial.print("acceleration reading = ");
    // }
    // char* dataChar = (char*)pData;
    digitalWrite(ledPin, HIGH);
    digitalWrite(ledGreen, HIGH);
    Serial.write(pData, length);
    client.publish("test/status", pData, length);
    Serial.println();
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    digitalWrite(ledYellow, HIGH);
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    digitalWrite(ledYellow, LOW);
    digitalWrite(ledGreen, LOW);
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");
    pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(accelerometerSensorServiceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(accelerometerSensorServiceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristicX = pRemoteService->getCharacteristic(acclerometerCharacteristicUUID);
    if (pRemoteCharacteristicX == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(acclerometerCharacteristicUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");


    // Read the value of the characteristic.
    if(pRemoteCharacteristicX->canRead()) {
      int value = pRemoteCharacteristicX->readUInt8();
      // Serial.print("The characteristic value was: ");
      // Serial.println(value);
      // Serial.println("TESTvalue");
    }


    if(pRemoteCharacteristicX->canNotify())
      pRemoteCharacteristicX->registerForNotify(notifyCallback);


    connected = true;
    return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(accelerometerSensorServiceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks


void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");

  pinMode(ledPin, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledYellow, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  digitalWrite(ledPin, LOW);
  digitalWrite(ledRed, LOW);
  digitalWrite(ledGreen, LOW);
  digitalWrite(ledYellow, LOW);
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.

  setupWiFi();
  client.setServer(mqttServer, mqttPort);


} // End of setup.

int countBLE = 0;

// This is the Arduino main loop function.
void loop() {

  while (!client.connected()) {
    digitalWrite(ledPin, LOW);
    digitalWrite(ledRed, LOW);
    digitalWrite(ledYellow, LOW);
    digitalWrite(ledGreen, LOW);
    reconnect();
  }

  if (client.connected()) {
    digitalWrite(ledRed, HIGH);
  }

  if (countBLE == 0) {
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(7, false);
    countBLE = 1;
  }

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  while (doConnect == true && client.connected()) {
    Serial.println("trying to connect...");
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
      digitalWrite(ledPin, LOW);
    }
    doConnect = false;
  }

  client.loop();
  

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
    String newValue = String(millis()/1000);
    // Serial.println("Setting new characteristic value to \"" + newValue + "\"");
    
    // Set the characteristic's value to be the array of bytes that is actually a string.
    pRemoteCharacteristicX->writeValue(newValue.c_str(), newValue.length());
    digitalWrite(ledYellow, HIGH);
  }else if(doScan){
    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
    digitalWrite(ledYellow, LOW);
  }
  
  // delay(1000); // Delay a second between loops.
} // End of loop