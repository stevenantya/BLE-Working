/*
 This code is designed for OTTER System developed by the National University of Singapore, Waskito, Leow, Medaranga, Varshney, 2023.
 
 This code interfaces with 4 sensors embedded or externally connected to Arduino Nano 33 BLE Sense. The sensors are:
 1. Altimeter and Temperature MPL3115A2
 2. Air Quality Sensor Sensirion SEN54
 3. Microphone MAX9814
 4. Light Sensor

*/
isBLEExperiment = true;
#include<Arduino.h>
#include <ArduinoBLE.h>
#include <avr/dtostrf.h>
#include<Adafruit_MPL3115A2.h>
#include <SensirionI2CSen5x.h>
#include <Arduino_APDS9960.h>
#include <Wire.h>

Adafruit_MPL3115A2 baro;
SensirionI2CSen5x sen5x;

#define MAXBUF_REQUIREMENT 48

#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#define USE_PRODUCT_INFO
#endif

//BLE 

BLEService proximityColorSensorService("7cf0c267-4712-4ad8-bd08-47c1e7ed5e34");
BLEService airQualitySensorService("7cf0c267-4712-4ad8-bd08-47c1e7ed5e35");
BLEService microphoneSensorService("7cf0c267-4712-4ad8-bd08-47c1e7ed5e36");
BLEService altimeterTemperatureSensorService("7cf0c267-4712-4ad8-bd08-47c1e7ed5e37");

//Ligth Sensor Variables
int proximity = 0;
int r = 0, g = 0, b = 0;
char proximityColorData[50] = "0000,0000,0000,0000\n";

//Air Quality Sensor Variables
char airQualityData[50] = "00.00,00.00,00.00,00.00,00,00.00,00.00\n";

//Microphone Variables
char microphoneData[50] = "0000.0000\n";

//Barometer Variables
char altimeterTemperatureData[50] = "0000.00,00.00,00.00\n";

BLECharacteristic proximityColorSensor("6ad7b9c7-5f64-48f2-8a3b-2aeb886145b8",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, proximityColorData); // remote clients will be able to get notifications if this characteristic changes
BLECharacteristic airQualitySensor("6ad7b9c7-5f64-48f2-8a3b-2aeb886145b9",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, airQualityData); // remote clients will be able to get notifications if this characteristic changes
BLECharacteristic microphoneSensor("6ad7b9c7-5f64-48f2-8a3b-2aeb886145ba",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, microphoneData); // remote clients will be able to get notifications if this characteristic changes
BLECharacteristic altimeterTemperatureSensor("6ad7b9c7-5f64-48f2-8a3b-2aeb886145bb",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, altimeterTemperatureData); // remote clients will be able to get notifications if this characteristic changes


void printModuleVersions() {
    uint16_t error;
    char errorMessage[256];

    unsigned char productName[32];
    uint8_t productNameSize = 32;

    error = sen5x.getProductName(productName, productNameSize);

    if (error) {
        Serial.print("Error trying to execute getProductName(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("ProductName:");
        Serial.println((char*)productName);
    }

    uint8_t firmwareMajor;
    uint8_t firmwareMinor;
    bool firmwareDebug;
    uint8_t hardwareMajor;
    uint8_t hardwareMinor;
    uint8_t protocolMajor;
    uint8_t protocolMinor;

    error = sen5x.getVersion(firmwareMajor, firmwareMinor, firmwareDebug,
                             hardwareMajor, hardwareMinor, protocolMajor,
                             protocolMinor);
    if (error) {
        Serial.print("Error trying to execute getVersion(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("Firmware: ");
        Serial.print(firmwareMajor);
        Serial.print(".");
        Serial.print(firmwareMinor);
        Serial.print(", ");

        Serial.print("Hardware: ");
        Serial.print(hardwareMajor);
        Serial.print(".");
        Serial.println(hardwareMinor);
    }
}

void printSerialNumber() {
    uint16_t error;
    char errorMessage[256];
    unsigned char serialNumber[32];
    uint8_t serialNumberSize = 32;

    error = sen5x.getSerialNumber(serialNumber, serialNumberSize);
    if (error) {
        Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("SerialNumber:");
        Serial.println((char*)serialNumber);
    }
}

float getMicrophoneData() {
    const int microphonePin = A0;
    const int numReadings = 50;

    const float Vref = 3.3;  // Reference voltage for Nano BLE 33 Sense
    const float Vmin = 0.006;  // Minimum detectable voltage, can be adjusted

    long total = 0;
  
    for (int i = 0; i < numReadings; i++) {
        total += analogRead(microphonePin);
        delay(1);
    }
    
    int averageValue = total / numReadings;
    float Vout = (averageValue / 1023.0) * Vref;
    
    // Check for Vout being below Vmin to avoid log(0)
    if (Vout < Vmin) {
        Vout = Vmin;
    }
    
    float dB = 20 * log10(Vout / Vmin);

    return dB;
}

// void printSerial has pressure, altitude, temperature, massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0, massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex, noxIndex, proximity, r, g, b, microphoneData) as parameters
void printSerial(float pressure, altitude, temperature, massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0, massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex, noxIndex, int proximity, int r, int g, int b, int microphoneData) {
    //print all of the sensor values above
    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.print(" Pa, Altitude: ");
    Serial.print(altitude);
    Serial.print(" m, Temperature: ");
    Serial.print(temperature);
    Serial.println(" deg. C");

    //print all of the air quality values
    Serial.print("Mass Concentration PM 1.0: ");
    Serial.print(massConcentrationPm1p0);
    Serial.print(" ug/m3, PM 2.5: ");
    Serial.print(massConcentrationPm2p5);
    Serial.print(" ug/m3, PM 4.0: ");
    Serial.print(massConcentrationPm4p0);
    Serial.print(" ug/m3, PM 10.0: ");
    Serial.print(massConcentrationPm10p0);
    Serial.print(" ug/m3, Ambient Humidity: ");
    Serial.print(ambientHumidity);
    Serial.print(" %, Ambient Temperature: ");
    Serial.print(ambientTemperature);
    Serial.print(" deg. C, VOC Index: ");
    Serial.print(vocIndex);
    Serial.print(", NOx Index: ");
    Serial.println(noxIndex);

    //print all of the light sensor values
    Serial.print("Proximity: ");
    Serial.print(proximity);
    Serial.print(", Red: ");
    Serial.print(r);
    Serial.print(", Green: ");
    Serial.print(g);
    Serial.print(", Blue: ");
    Serial.println(b);
}

void updateBarometerData() {
    float pressure = baro.getPressure();
    float altitude = baro.getAltitude();
    float temperature = baro.getTemperature();

    char char_pressure[7] = "0000.00";
    char char_altitude[7] = "0000.00";
    char char_temperature[7] = "00.00";

    dtostrf(pressure, 0, 2, char_pressure);
    dtostrf(altitude, 0, 2, char_altitude);
    dtostrf(temperature, 0, 2, char_temperature);

    strcpy(altimeterTemperatureData, char_pressure);
    strcat(altimeterTemperatureData, ",");
    strcat(altimeterTemperatureData, char_altitude);
    strcat(altimeterTemperatureData, ",");
    strcat(altimeterTemperatureData, char_temperature);

    altimeterTemperatureSensor.writeValue(altimeterTemperatureData);
}

void updateAirQualityData() {
    uint16_t error;
    char errorMessage[256];
    // Read Measurement
    float massConcentrationPm1p0;
    float massConcentrationPm2p5;
    float massConcentrationPm4p0;
    float massConcentrationPm10p0;
    float ambientHumidity;
    float ambientTemperature;
    float vocIndex;
    float noxIndex;

    error = sen5x.readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);

    if (error) {
        Serial.print("Error trying to execute readMeasuredValues(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    //create char conversion using dtostrf for all variables above
    char char_massConcentrationPm1p0[7] = "00.00";
    char char_massConcentrationPm2p5[7] = "00.00";
    char char_massConcentrationPm4p0[7] = "00.00";
    char char_massConcentrationPm10p0[7] = "00.00";
    char char_ambientHumidity[7] = "00.00";
    char char_ambientTemperature[7] = "00.00";
    char char_vocIndex[7] = "00.00";

    //convert all variables above to char
    dtostrf(massConcentrationPm1p0, 0, 2, char_massConcentrationPm1p0);
    dtostrf(massConcentrationPm2p5, 0, 2, char_massConcentrationPm2p5);
    dtostrf(massConcentrationPm4p0, 0, 2, char_massConcentrationPm4p0);
    dtostrf(massConcentrationPm10p0, 0, 2, char_massConcentrationPm10p0);
    dtostrf(ambientHumidity, 0, 2, char_ambientHumidity);
    dtostrf(ambientTemperature, 0, 2, char_ambientTemperature);
    dtostrf(vocIndex, 0, 2, char_vocIndex);

    //concatenate all char variables above into one char variable
    strcpy(airQualityData, char_massConcentrationPm1p0);
    strcat(airQualityData, ",");
    strcat(airQualityData, char_massConcentrationPm2p5);
    strcat(airQualityData, ",");
    strcat(airQualityData, char_massConcentrationPm4p0);
    strcat(airQualityData, ",");
    strcat(airQualityData, char_massConcentrationPm10p0);
    strcat(airQualityData, ",");
    strcat(airQualityData, char_ambientHumidity);
    strcat(airQualityData, ",");
    strcat(airQualityData, char_ambientTemperature);
    strcat(airQualityData, ",");
    strcat(airQualityData, char_vocIndex);

    //write the char variable to the BLE characteristic
    airQualitySensor.writeValue(airQualityData);

}

void updateLightSensorData() {
    int proximity,r,g,b;
    proximity = APDS.readProximity();
    APDS.readColor(r, g, b);

    //create char conversion using dtostrf for all variables above
    char char_proximity[7] = "0000";
    char char_r[7] = "0000";
    char char_g[7] = "0000";
    char char_b[7] = "0000";

    //convert all variables above to char
    itoa(proximity,char_proximity,10);
    itoa(r,char_r,10);
    itoa(g,char_g,10);
    itoa(b,char_b,10);

    //concatenate all char variables above into one char variable
    strcpy(proximityColorData, char_proximity);
    strcat(proximityColorData, ",");
    strcat(proximityColorData, char_r);
    strcat(proximityColorData, ",");
    strcat(proximityColorData, char_g);
    strcat(proximityColorData, ",");
    strcat(proximityColorData, char_b);

    //write the char variable to the BLE characteristic
    proximityColorSensor.writeValue(proximityColorData);
}

void updateMicrophoneData() {
    float microphoneData = getMicrophoneData();

    //create char conversion using dtostrf for all variables above
    char char_microphoneData[7] = "0000.00";

    //convert all variables above to char
    dtostrf(microphoneData, 0, 2, char_microphoneData);

    //concatenate all char variables above into one char variable
    strcpy(microphoneData, char_microphoneData);

    //write the char variable to the BLE characteristic
    microphoneSensor.writeValue(microphoneData);
}

void setup() {
    Serial.begin(115200);
    while (!Serial){
        delay(100);
    }

    //BAROMETER
    if (!baro.begin()) {
        Serial.println("Couldnt find sensor");
        while (1);
    } else {
        Serial.println("Found MPL3115A2 sensor");
    }
    baro.setSeaPressure(1012);


    //AIR QUALITY SENSOR
    Wire.begin();
    sen5x.begin(Wire);

    uint16_t error;
    char errorMessage[256];
    error = sen5x.deviceReset();
    if (error) {
        Serial.print("Error trying to execute deviceReset(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    #ifdef USE_PRODUCT_INFO
        printSerialNumber();
        printModuleVersions();
    #endif

    float tempOffset = 0.0;
    error = sen5x.setTemperatureOffsetSimple(tempOffset);
    if (error) {
        Serial.print("Error trying to execute setTemperatureOffsetSimple(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("Temperature Offset set to ");
        Serial.print(tempOffset);
        Serial.println(" deg. Celsius (SEN54/SEN55 only");
    }

    // Start Measurement
    error = sen5x.startMeasurement();
    if (error) {
        Serial.print("Error trying to execute startMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    //LIGHT SENSOR
    if (!APDS.begin()) {
        Serial.println("Error initializing APDS-9960 sensor.");
        while (true); // Stop forever
    }


    //BLE

    //check if connected or not
    while (!BLE.begin()) {
        Serial.println("starting BLE failed!");
    }
    /* Set a local name for the Bluetooth® Low Energy device
     This name will appear in advertising packets
     and can be used by remote devices to identify this Bluetooth® Low Energy device
     The name can be changed but maybe be truncated based on space left in advertisement packet
    */
    BLE.setLocalName("Arduino Nano 33");

    BLE.setAdvertisedService(proximityColorSensorService);
    proximityColorSensorService.addCharacteristic(proximityColorSensor);
    BLE.addService(proximityColorSensorService);

    BLE.setAdvertisedService(airQualitySensorService);
    airQualitySensorService.addCharacteristic(airQualitySensor);
    BLE.addService(airQualitySensorService);

    BLE.setAdvertisedService(microphoneSensorService);
    microphoneSensorService.addCharacteristic(microphoneSensor);
    BLE.addService(microphoneSensorService);

    BLE.setAdvertisedService(altimeterTemperatureSensorService);
    altimeterTemperatureSensorService.addCharacteristic(altimeterTemperatureSensor);
    BLE.addService(altimeterTemperatureSensorService);

    proximityColorSensor.writeValue(proximityColorData);
    airQualitySensor.writeValue(airQualityData);
    microphoneSensor.writeValue(microphoneData);
    altimeterTemperatureSensor.writeValue(altimeterTemperatureData);

     // start advertising
    BLE.advertise();

    Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {

    BLEDevice central = BLE.central();

    if (central) {
        Serial.print("Connected to central: ");
        // print the central's BT address:
        Serial.println(central.address());
        // turn on the LED to indicate the connection:
        digitalWrite(LED_BUILTIN, HIGH);
        delay(5000);
        
        bool isFirstConnection = true;
        while (central.connected()) {
            int curr_time = millis();
            if (isFirstConnection) {
                while (millis() < curr_time + 2000); //wait for 2 seconds for bluetooth to establish correctly
                isFirstConnection = false;
            }
            updateBarometerData();
            updateAirQualityData();
            updateLightSensorData();
            updateMicrophoneData();
            
            delay(1000);
        }
        // when the central disconnects, turn off the LED:
        digitalWrite(LED_BUILTIN, LOW);
        Serial.print("Disconnected from central: ");
        Serial.println(central.address());
    }
}