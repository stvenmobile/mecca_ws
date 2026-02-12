import serial
import time

# Quick hybrid test
ser = serial.Serial('/dev/stm32_serial', 115200, timeout=1)
time.sleep(2) # Wait for reboot

try:
    while True:
        # Send BOTH encoder request and movement command
        print("Sending: V 100 0 0 and I ENC")
        ser.write(b"V 100 0 0\r\n")
        ser.write(b"I ENC\r\n")
        ser.flush()
        
        line = ser.readline().decode().strip()
        if line:
            print(f"RX: {line}")
        time.sleep(0.1) # 10Hz - similar to ROS 2 rate
except KeyboardInterrupt:
    ser.write(b"V 0 0 0\r\n")
    ser.close()
