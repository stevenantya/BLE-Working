/*
  Battery Monitor

  This example creates a Bluetooth® Low Energy peripheral with the standard battery service and
  level characteristic. The A0 pin is used to calculate the battery level.

  The circuit:
  - Arduino MKR WiFi 1010, Arduino Uno WiFi Rev2 board, Arduino Nano 33 IoT,
    Arduino Nano 33 BLE, or Arduino Nano 33 BLE Sense board.

  You can use a generic Bluetooth® Low Energy central app, like LightBlue (iOS and Android) or
  nRF Connect (Android), to interact with the services and characteristics
  created in this sketch.

  This example code is in the public domain.
*/

#include <ArduinoBLE.h>
#include <avr/dtostrf.h>
#include <Arduino_LSM9DS1.h>
#include <Arduino_APDS9960.h>

 // Bluetooth® Low Energy Battery Service
BLEService sensorService("4fafc201-1fb5-459e-8fcc-c5c9c331914b");

//Accelerometer Variables
char acc_x[7] = "-00.00";
char acc_y[7] = "-00.00";
char acc_z[7] = "-00.00";
char accData[150] = "-00.00,-00.00,-0.00\n-00.00,-00.00,-00.00\n-00.00,-00.00,-00.00\n-00.00,-00.00,-00.00\n-00.00,-00.00,-00.00\n";

//Light Sensor Variables
int proximity = 0;
int r = 0, g = 0, b = 0;
unsigned long lastUpdate = 0;
char proximityColorData[150] = "0";

// Bluetooth® Low Energy Battery Level Characteristic
BLECharacteristic accelerometerSensor("beb5483e-36e1-4688-b7f5-ea07361b26a8",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, accData); // remote clients will be able to get notifications if this characteristic changes
BLECharacteristic proximityColorSensor("6ad7b9c7-5f64-48f2-8a3b-2aeb886145b8",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, proximityColorData); // remote clients will be able to get notifications if this characteristic changes

void setup() {
  Serial.begin(115200);    // initialize serial communication
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  if (!APDS.begin()) {
    Serial.println("Error initializing APDS-9960 sensor.");
    while (true); // Stop forever
  }

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  /* Set a local name for the Bluetooth® Low Energy device
     This name will appear in advertising packets
     and can be used by remote devices to identify this Bluetooth® Low Energy device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("Accelerometer Nano 33");
  BLE.setAdvertisedService(sensorService); // add the service UUID
  sensorService.addCharacteristic(accelerometerSensor); // add the battery level characteristic
  sensorService.addCharacteristic(proximityColorSensor);

  BLE.addService(sensorService); // Add the battery service
  accelerometerSensor.writeValue(acc_x); // set initial value for this characteristic
  proximityColorSensor.writeValue(proximityColorData);

  /* Start advertising Bluetooth® Low Energy.  It will start continuously transmitting Bluetooth® Low Energy
     advertising packets and will be visible to remote Bluetooth® Low Energy central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);
    delay(5000);
    
    // while the central is connected:
    bool isFirstConnection = true;
    while (central.connected()) {
      curr_time = millis();
      if (isFirstConnection) {
        while (millis() < curr_time + 2000); //wait for 2 seconds for bluetooth to establish correctly
        isFirstConnection = false;
      }
      // if 200ms have passed, check the battery level:
      if (IMU.accelerationAvailable()) {
        updateAccelerometer();
      }

      if (APDS.proximityAvailable() && APDS.colorAvailable) {
        updateProximityColorSensor();
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void updateProximityColorSensor() {
  proximity = APDS.readProximity();
  APDS.readColor(r, g, b);
}

int internal_counter = 0;
char final[150];

void updateAccelerometer() {

  float x,y,z;
  IMU.readAcceleration(x, y, z);
  
  dtostrf((double)x, 2, 2, acc_x);
  dtostrf((double)y, 2, 2, acc_y);
  dtostrf((double)z, 2, 2, acc_z);

  strcpy(accData, acc_x);
  strcat(accData," ");
  strcat(accData, acc_y);
  strcat(accData," ");
  strcat(accData, acc_z);

  if (internal_counter >= 4) {
    Serial.println(final);
    accelerometerSensor.writeValue(final);  // and update the battery level characteristic

    internal_counter = 0;
    strcpy(final, accData);
    strcat(final, "\n");
  }
  else {
    internal_counter += 1;
    strcat(final, accData);
    strcat(final, "\n");
  }
  
}
