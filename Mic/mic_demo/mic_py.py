import serial
import wave

PORT = 'COM5' # Replace with your Arduino's serial port (e.g., COM3 or /dev/ttyUSB0)
BAUD_RATE = 115200
DURATION = 10  # Recording duration in seconds
SAMPLE_RATE = 4000  # Sample rate in Hz (this is a rough approximation)

ser = serial.Serial(PORT, BAUD_RATE)

# Initialize an empty list to store the samples
samples = []

# Read audio samples for the specified duration
for _ in range(SAMPLE_RATE * DURATION):
    try:
        line = ser.readline().decode('utf-8').strip()  # Read a line from the serial port
        sample = int(line)
        samples.append(sample)
    except:
        pass

ser.close()

# Convert samples to bytes
samples_bytes = bytes(samples)

# Write to a wave file
with wave.open("audio_output.wav", "wb") as wave_file:
    wave_file.setnchannels(1)
    wave_file.setsampwidth(2)  # 2 bytes
    wave_file.setframerate(SAMPLE_RATE)
    wave_file.writeframes(samples_bytes)