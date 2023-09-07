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
const char* mqttServer = "192.168.3.135";
const int mqttPort = 1883;
const char* mqttUser = "pi";  // If needed
const char* mqttPassword = "123456"; // If needed
const char* mqttTopic = "robot";

WiFiClient espClient;
PubSubClient client(espClient);

//Proximity Color Sensor
// The remote service we wish to connect to.
static BLEUUID proximityColorSensorServiceUUID("7cf0c267-4712-4ad8-bd08-47c1e7ed5e34");
// The characteristic of the remote service we are interested in.
static BLEUUID proximityColorCharacteristicUUID("6ad7b9c7-5f64-48f2-8a3b-2aeb886145b8");

//Air Quality Sensor
static BLEUUID airQualitySensorServiceUUID("7cf0c267-4712-4ad8-bd08-47c1e7ed5e35");
static BLEUUID airQualityCharacteristicUUID("6ad7b9c7-5f64-48f2-8a3b-2aeb886145b9");

//Microphone Sensor
static BLEUUID microphoneSensorServiceUUID("7cf0c267-4712-4ad8-bd08-47c1e7ed5e36");
static BLEUUID microphoneCharacteristicUUID("6ad7b9c7-5f64-48f2-8a3b-2aeb886145b0");

//Altimeter Temperature Sensor
static BLEUUID altimeterTemperatureSensorServiceUUID("7cf0c267-4712-4ad8-bd08-47c1e7ed5e37");
static BLEUUID altimeterTemperatureCharacteristicUUID("6ad7b9c7-5f64-48f2-8a3b-2aeb886145b1");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristicProx;
static BLERemoteCharacteristic* pRemoteCharacteristicAir;
static BLERemoteCharacteristic* pRemoteCharacteristicMic;
static BLERemoteCharacteristic* pRemoteCharacteristicAlt;
static BLEAdvertisedDevice* myDevice;

//MQTT
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

//MQTT
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
//MQTT Send Message
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    // Serial.print(millis());
    // Serial.print("  ");
    // Serial.print("Notify callback for characteristic ");
    String identifier = pBLERemoteCharacteristic->getUUID().toString().c_str();
    if (identifier.equals("6ad7b9c7-5f64-48f2-8a3b-2aeb886145b8")) {
      //Light Sensor
      client.publish("light", pData, length);
    } else if (identifier.equals("6ad7b9c7-5f64-48f2-8a3b-2aeb886145b9")) {
      //Air Quality Sensor
      client.publish("air", pData, length);
    } else if (identifier.equals("6ad7b9c7-5f64-48f2-8a3b-2aeb886145b0")) {
      //Microphone Sensor
      client.publish("microphone", pData, length);
    } else if (identifier.equals("6ad7b9c7-5f64-48f2-8a3b-2aeb886145b1")) {
      //Altimeter Temperature Sensor
      client.publish("temperature", pData, length);
    } else {
      //Other Sensor
      client.publish("other", pData, length);
    }
    
    // char* dataChar = (char*)pData;
    digitalWrite(ledPin, HIGH);
    digitalWrite(ledGreen, HIGH);
    Serial.write(pData, length);

    client.publish("robot", pData, length);
    Serial.println();
}

//BLE
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

//BLE
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

    // Proxmity and Color Sensor Service and Characteristic

    BLERemoteService* pRemoteServiceProx = pClient->getService(proximityColorSensorServiceUUID);
    if (pRemoteServiceProx == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(proximityColorSensorServiceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristicProx = pRemoteServiceProx->getCharacteristic(proximityColorCharacteristicUUID);
    if (pRemoteCharacteristicProx == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(proximityColorCharacteristicUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");


    // Read the value of the characteristic.
    if(pRemoteCharacteristicProx->canRead()) {
      int value = pRemoteCharacteristicProx->readUInt8();
      // Serial.print("The characteristic value was: ");
      // Serial.println(value);
      // Serial.println("TESTvalue");
    }


    if(pRemoteCharacteristicProx->canNotify())
      pRemoteCharacteristicProx->registerForNotify(notifyCallback);


    // Air Quality Sensor Service and Characteristic
    BLERemoteService* pRemoteServiceAir = pClient->getService(airQualitySensorServiceUUID);
    if (pRemoteServiceAir == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(airQualitySensorServiceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristicAir = pRemoteServiceAir->getCharacteristic(airQualityCharacteristicUUID);
    if (pRemoteCharacteristicAir == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(airQualityCharacteristicUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    if (pRemoteCharacteristicAir->canRead()) {
      int value = pRemoteCharacteristicAir->readUInt8();
    }

    if (pRemoteCharacteristicAir->canNotify())
      pRemoteCharacteristicAir->registerForNotify(notifyCallback);

    // Microphone Sensor Service and Characteristic
    BLERemoteService* pRemoteServiceMic = pClient->getService(microphoneSensorServiceUUID);

    if (pRemoteServiceMic == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(microphoneSensorServiceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristicMic = pRemoteServiceMic->getCharacteristic(microphoneCharacteristicUUID);
    if (pRemoteCharacteristicMic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(microphoneCharacteristicUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    if (pRemoteCharacteristicMic->canRead()) {
      int value = pRemoteCharacteristicMic->readUInt8();
    }

    if (pRemoteCharacteristicMic->canNotify())
      pRemoteCharacteristicMic->registerForNotify(notifyCallback);

    // Altimeter Temperature Sensor Service and Characteristic
    BLERemoteService* pRemoteServiceAlt = pClient->getService(altimeterTemperatureSensorServiceUUID);

    if (pRemoteServiceAlt == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(altimeterTemperatureSensorServiceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristicAlt = pRemoteServiceAlt->getCharacteristic(altimeterTemperatureCharacteristicUUID);
    if (pRemoteCharacteristicAlt == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(altimeterTemperatureCharacteristicUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    if (pRemoteCharacteristicAlt->canRead()) {
      int value = pRemoteCharacteristicAlt->readUInt8();
    }

    if (pRemoteCharacteristicAlt->canNotify())
      pRemoteCharacteristicAlt->registerForNotify(notifyCallback);



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
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(proximityColorSensorServiceUUID) 
          || advertisedDevice.isAdvertisingService(airQualitySensorServiceUUID) 
          || advertisedDevice.isAdvertisingService(microphoneSensorServiceUUID) 
          || advertisedDevice.isAdvertisingService(altimeterTemperatureSensorServiceUUID)) {

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

  //UNCOMMENT FOR MQTT
  setupWiFi();
  client.setServer(mqttServer, mqttPort);


} // End of setup.

int countBLE = 0;

// This is the Arduino main loop function.
void loop() {

  //UNCOMMENT FOR MQTT
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
  //UNCOMMENT FOR MQTT
  while (doConnect == true && client.connected()) {
  // while (doConnect == true) {
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
    
    pRemoteCharacteristicProx->writeValue(newValue.c_str(), newValue.length());
    pRemoteCharacteristicAir->writeValue(newValue.c_str(), newValue.length());
    pRemoteCharacteristicMic->writeValue(newValue.c_str(), newValue.length());
    pRemoteCharacteristicAlt->writeValue(newValue.c_str(), newValue.length());
    
    digitalWrite(ledYellow, HIGH);
  }else if(doScan){
    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
    digitalWrite(ledYellow, LOW);
  }
  
  // delay(1000); // Delay a second between loops.
} // End of loop
