import serial
import numpy as np
from scipy.io.wavfile import write

SAMPLE_RATE = 4000  # Must match the Arduino's sample rate
DURATION = 10  # Duration of recording in seconds

# Set up serial connection (modify the COM port accordingly)
ser = serial.Serial('COM5', 250000)
data = []

print("Recording...")
for _ in range(SAMPLE_RATE * DURATION):
    try:
        line = ser.readline().decode('utf-8').strip()
        value = int(line)
        data.append(value)
    except:
        pass

print("Saving...")
data = np.array(data, dtype=np.int16)
write("C:/Users/steve/NUS_NCS_IOT/BLE Working/Mic/mic_demo/output.wav", SAMPLE_RATE, data)

ser.close()