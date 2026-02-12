import serial
import time

# Use the port connected to the STM32
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def move_m1_only():
    print("Reading current encoder values...")
    # This specific vector should result in M1 spinning while others stay near zero
    # Adjust values if your internal APB scaling differs
    ser.write(b"I ENC\r\n") 
    print("Encoders read complete.")

if __name__ == "__main__":
    move_m1_only()
