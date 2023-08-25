const int MIC_PIN = A0;  // Set to the analog pin where your microphone's OUT is connected
const int SAMPLE_RATE = 10000;  // Sample rate in Hz. You can adjust this based on your needs.

void setup() {
  Serial.begin(115200);
  analogReadResolution(10);  // Set ADC resolution to 10 bits for Nano 33 BLE Sense
}

void loop() {
  int micValue = analogRead(MIC_PIN);
  Serial.println(micValue);

  delayMicroseconds(1000000 / SAMPLE_RATE);  // Respect the sample rate
}
