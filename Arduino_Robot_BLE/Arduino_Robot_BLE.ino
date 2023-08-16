/*
 This code is designed for OTTER System developed by the National University of Singapore, Waskito, Leow, Medaranga, Varshney, 2023.
 
 This code interfaces with 4 sensors embedded or externally connected to Arduino Nano 33 BLE Sense. The sensors are:
 1. Altimeter and Temperature MPL3115A2
 2. Air Quality Sensor Sensirion SEN54
 3. Microphone MAX9814
 4. Light Sensor

*/

#include<Arduino.h>
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
}

void loop() {
    //BAROMETER
    float pressure = baro.getPressure();
    float altitude = baro.getAltitude();
    float temperature = baro.getTemperature();


    //AIR QUALITY SENSOR
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


    //LIGHT SENSOR
    int proximity,r,g,b;
    if (APDS.proximityAvailable() && APDS.colorAvailable()) {
        proximity = APDS.readProximity();
        APDS.readColor(r, g, b);
    }

    //MICROPHONE
    int dummyVolume = 10;


    //PRINTING and TRANSMITTING
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
    
    
    delay(1000);
}