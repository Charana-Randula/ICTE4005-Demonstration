import serial
import time

# Specify the correct serial port and baud rate
# Replace 'COM3' with the port your Arduino is connected to (e.g., 'COM3', '/dev/ttyUSB0', etc.)
ser = serial.Serial('/dev/cu.usbmodem1101', 115200, timeout=1)

# Give some time for the serial connection to initialize
time.sleep(2)

# Loop to continuously read data from Arduino
try:
    while True:
        if ser.in_waiting > 0:
            # Read the line from serial and decode it using UTF-8 encoding
            data = ser.readline().decode('utf-8').strip()
            if data:
                print(f"Data received: {data}")

except KeyboardInterrupt:
    print("Exiting...")

finally:
    ser.close()  # Make sure to close the serial port when done