import serial
import time

# Use the port connected to the STM32
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def move_m1_only():
    print("Sending reset command...")
    # This specific vector should result in M1 spinning while others stay near zero
    # Adjust values if your internal APB scaling differs
    ser.write(b"I RESET\r\n") 
    print("Encoders reset complete.")

if __name__ == "__main__":
    move_m1_only()
