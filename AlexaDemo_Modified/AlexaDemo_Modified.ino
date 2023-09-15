#include "Arduino.h"
#include "NDP.h"

//const bool lowestPower = true;
const bool lowestPower = false;

uint8_t data[2048];

void ledBlueOn(char* label) {
  nicla::leds.begin();
  nicla::leds.setColor(blue);
  delay(200);
  nicla::leds.setColor(off);
  if (!lowestPower) {
    Serial.println(label);
  }
  nicla::leds.end();
}

void ledGreenOn() {
  nicla::leds.begin();
  nicla::leds.setColor(green);
  delay(200);
  nicla::leds.setColor(off);
  nicla::leds.end();
}

void ledRedBlink() {
  while (1) {
    nicla::leds.begin();
    nicla::leds.setColor(red);
    delay(200);
    nicla::leds.setColor(off);
    delay(200);
    nicla::leds.end();
  }
}

void setup() {

  Serial.begin(115200);
  nicla::begin();
  nicla::disableLDO();
  nicla::leds.begin();

  NDP.onError(ledRedBlink);
  NDP.onMatch(ledBlueOn);
  NDP.onEvent(ledGreenOn);
  // Serial.println("Loading synpackages");
  NDP.begin("mcu_fw_120_v91.synpkg");
  NDP.load("dsp_firmware_v91.synpkg");
  NDP.load("alexa_334_NDP120_B0_v11_v91.synpkg");
  // Serial.println("packages loaded");
  // NDP.getInfo();
  // Serial.println("Configure mic");
  NDP.turnOnMicrophone();

  // Check the audio chunk size to ensure it doesn't exceed the buffer size
  int chunk_size = NDP.getAudioChunkSize();
  if (chunk_size >= sizeof(data)) {
    for(;;);
  }

  // For maximum low power; please note that it's impossible to print after calling these functions
  nicla::leds.end();
  if (lowestPower) {
    NRF_UART0->ENABLE = 0;
  }
  //NDP.turnOffMicrophone();
}
const long SAMPLING_RATE = 4000;
const long INTERVAL = 1000000L / SAMPLING_RATE;
long previousMicros = 0;

void loop() {
  long currentMicros = micros();

  if (currentMicros - previousMicros >= INTERVAL) {
    previousMicros = currentMicros;
    unsigned int len = 0;
    NDP.extractData(data, &len);
    Serial.write(data, len);
  }
}