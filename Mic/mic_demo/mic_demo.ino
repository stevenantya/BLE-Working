const int microphonePin = A0;
const long INTERVAL = 250; // Time in microseconds for 4kHz sample rate

long previousMicros = 0;

void setup() {
  Serial.begin(115200); // Begin the serial communication
}

void loop() {
  long currentMicros = micros();
  
  if (currentMicros - previousMicros >= INTERVAL) {
    previousMicros = currentMicros;
    
    int audioValue = analogRead(microphonePin);
    Serial.println(audioValue);
  }
}