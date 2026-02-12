import serial
import time

# Use the port connected to the STM32
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

def move_m1_only():
    print("Sending targeted M1 (Right Rear) test...")
    # This specific vector should result in M1 spinning while others stay near zero
    # Adjust values if your internal APB scaling differs
    ser.write(b"V 100 -100 25\r\n") 
    time.sleep(5)
    ser.write(b"V 0 0 0\r\n")
    print("Test complete. Check M1 wheel.")

if __name__ == "__main__":
    move_m1_only()
