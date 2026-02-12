import serial
import time

def main():
    port = '/dev/stm32_serial'
    baud = 115200
    
    print(f"--- Mecca Motor Serial Test ---")
    
    try:
        # Open serial port with 1s timeout
        ser = serial.Serial(port, baud, timeout=1)
        print(f"Connected to {port} at {baud} baud.")
        
        # Give the STM32 a moment to initialize after the serial port opens
        # (This prevents commands being lost during a DTR-triggered reset)
        time.sleep(2)

        # 1. Move Forward
        print("Moving FORWARD (V 100 0 0)...")
        ser.write(b"V 100 0 0\r\n")
        ser.flush()
        time.sleep(2)

        # 2. Stop
        print("STOPPING (V 0 0 0)...")
        ser.write(b"V 0 0 0\r\n")
        ser.flush()
        time.sleep(1)

        # 3. Move Backward
        print("Moving BACKWARD (V -100 0 0)...")
        ser.write(b"V -100 0 0\r\n")
        ser.flush()
        time.sleep(2)

        # 4. Final Stop
        print("Final STOP (V 0 0 0)...")
        ser.write(b"V 0 0 0\r\n")
        ser.flush()

        print("Test Complete.")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Port closed.")

if __name__ == "__main__":
    main()
