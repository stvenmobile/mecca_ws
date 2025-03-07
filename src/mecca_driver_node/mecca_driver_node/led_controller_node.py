import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mecca_driver_node import ws2812
import spidev
import time
import numpy as np
from numpy import sin, pi
import threading

# Define LED strip configuration
LED_COUNT = 7  # Number of LEDs in your strip
SPI_BUS = 0     # SPI bus number
SPI_DEVICE = 0  # SPI device number

class LEDControllerNode(Node):
    def __init__(self):
        super().__init__('led_controller_node')

        # Define LED count properly
        self.num_leds = LED_COUNT  # Use the global constant defined at the top

        # Initialize SPI for LED control
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = 6400000  # 8MHz SPI clock

        self.get_logger().info("LED Controller Node Started")

        # Run startup sequence
        self.startup_sequence()

        # Set initial state to stopped (white lights)
        self.set_leds((255, 255, 255))

        # ROS2 subscriber for LED commands
        self.create_subscription(String, 'led_commands', self.led_command_callback, 10)


    def led_command_callback(self, msg):
        """Handle incoming commands and trigger corresponding LED effects."""
        command = msg.data.strip().lower()
        if command == "fwd":
            self.set_forward_effect()
        elif command == "bwd":
            self.set_backward_effect()
        elif command == "left":
            self.set_turn_effect("LEFT")
        elif command == "right":
            self.set_turn_effect("RIGHT")
        elif command == "stop":
            self.set_stop_effect()
        elif command == "start":
            self.startup_sequence()
        elif command == "rainbow":
            self.set_rainbow_wave_effect()
        elif command == "test":
            self.run_test_sequence()
        else:
            self.get_logger().warn(f"Unknown LED command: {command}")


    def startup_sequence(self):
        self.set_rainbow_wave_effect()
        time.sleep(2)

    def on_shutdown(self):
        """Turn off LEDs when the node shuts down"""
        self.set_leds((0, 0, 0))  # Set all LEDs to black (off)
        self.spi.close() # close the SPI channel gracefully
        self.get_logger().info("LEDs turned off, SPI channel closed on shutdown.")


    def set_forward_effect(self):
        """ Blue strobe: LEDs light from center to edges, then back to center """
        led_array = [(0, 0, 255)] * LED_COUNT  # Use the global LED_COUNT
        for i in range(self.num_leds // 2):
            led_array[i] = (0, 0, 0)  # Black (off)
            led_array[-(i + 1)] = (0, 0, 0)  # Black (off)
            self.update_strip(led_array)
            time.sleep(0.1)  # Fast effect

        # Reverse (back to full blue)
        for i in range(self.num_leds // 2):
            led_array[i] = (0, 0, 255)
            led_array[-(i + 1)] = ( 0, 0, 255)
            self.update_strip(led_array)
            time.sleep(0.1)  # Fast effect

    
    def set_backward_effect(self):
        """BWD: Flashes red at 0.5s intervals for 2 seconds."""
        red_array = [(255, 0, 0)] * LED_COUNT  
        off_array = [(0, 0, 0)] * LED_COUNT
        end_time = time.time() + 1.0
        while time.time() < end_time:
            self.set_leds((255, 0, 0))  # Red
            self.update_strip(red_array)
            time.sleep(0.5)
            self.set_leds((0, 0, 0))    # Off
            self.update_strip(off_array)
            time.sleep(0.3)
        self.set_leds((0, 0, 0))
        self.update_strip(off_array)

    
    def set_turn_effect(self, direction):
        """ Green scrolling effect: moves left/right based on turn direction """
        off_array = [(0, 0, 0)] * LED_COUNT
        end_time = time.time() + 1.0
        # Stop any previously running turn effect
        time.sleep(0.1)  # Allow any old animation to stop
        led_array = [(0, 0, 0)] * LED_COUNT  # Start with all off

        if direction == "LEFT":
            while time.time() < end_time:
                for i in range(self.num_leds):
                    led_array[i] = (0, 255, 0)  # Green
                    self.update_strip(led_array)
                    time.sleep(0.10)  # Adjust scrolling speed
                #self.set_leds((0, 0, 0))
                self.update_strip([(0, 0, 0)] * self.num_leds)
 
        elif direction == "RIGHT":
            while time.time() < end_time:
                for i in reversed(range(self.num_leds)):
                    led_array[i] = (0, 255, 0)  # Green
                    self.update_strip(led_array)
                    time.sleep(0.10)  # Adjust scrolling speed
                #self.set_leds((0, 0, 0))
                self.update_strip([(0, 0, 0)] * self.num_leds)

        # **🚀 Exit condition: Turn off LEDs when turning stops**
        self.update_strip(off_array)  



    def set_stop_effect(self):
        """ Stop effect: Only first and last LED white to conserve power """

        self.turning = False  # Stop any running turn animation
        time.sleep(0.1)  # Small delay to ensure the previous animation stops

        led_array = [(0, 0, 0)] * self.num_leds  # Start with all off
        led_array[0] = (255, 255, 255)  # First LED White
        led_array[-1] = (255, 255, 255)  # Last LED White
        self.update_strip(led_array)

        # self.get_logger().info("Stopped all animations and reset to STOP effect.")


    def set_rainbow_wave_effect(self, duration=6):
        """ Rainbow wave effect that runs for a set duration and then turns off. """
        
        self.running_effect = False  # Stop any previously running effect
        time.sleep(0.1)  # Small delay to ensure old animation stops

        self.running_effect = True  # Indicate an active effect
        tStart = time.time()
        indices = 4 * np.array(range(self.num_leds), dtype=np.uint32) * pi / self.num_leds
        period0, period1, period2 = 2.0, 2.1, 2.2  # Slight offsets for smooth transitions

        def animate_rainbow():
            while self.running_effect and (time.time() - tStart < duration):
                t = time.time() - tStart
                f = np.zeros((self.num_leds, 3))

                # Generate sinusoidal color wave
                f[:, 0] = sin(2 * pi * t / period0 + indices)  # Red channel
                f[:, 1] = sin(2 * pi * t / period1 + indices)  # Green channel
                f[:, 2] = sin(2 * pi * t / period2 + indices)  # Blue channel

                # Normalize and scale intensity
                f = (255 * ((f + 1.0) / 2.0)).astype(np.uint8)

                # Convert to list and update LED strip
                self.update_strip([tuple(c) for c in f])

                time.sleep(0.05)  # Controls speed of the wave effect

            # Ensure LEDs turn off after effect completes
            self.update_strip([(0, 0, 0)] * self.num_leds)
            self.get_logger().info("Rainbow effect completed, LEDs turned off.")

        # Run the effect in a separate thread
        threading.Thread(target=animate_rainbow, daemon=True).start()

    
    def color_sequence(self):
        led_array = [(0, 0, 0)] * LED_COUNT 
        """Flash the LED strip Red → Blue → Green three times"""

        sequence = [(255, 0, 0), (0, 0, 255), (0, 255, 0)]  # Red → Blue → Green
        #self.get_logger().info(f"LED_COUNT: {LED_COUNT}")
        for _ in range(2):  # Three cycles
            for color in sequence:
                self.set_leds(color)
                self.update_strip([(color)] * LED_COUNT)  # Force update
                time.sleep(0.5)  # Half-second delay
            time.sleep(0.5)
        self.set_leds((0, 0, 0))    # Off
        self.update_strip(led_array)


    #########################################################
    # Test each light pattern for seven seconds in sequence #
    #########################################################
    def run_test_sequence(self):
        self.get_logger().info("Starting LED Test Sequence")
        self.color_sequence()
        time.sleep(3)

        # Rainbow effect
        self.set_rainbow_wave_effect()
        time.sleep(3)

        # FWD Effect
        self.set_forward_effect()
        time.sleep(3)

        # BWD Effect
        self.set_backward_effect()
        time.sleep(3)

        # LEFT Turn Effect
        self.set_turn_effect("LEFT")
        time.sleep(3)
        self.set_stop_effect()  # Stop turning before switching effects

        # RIGHT Turn Effect
        self.set_turn_effect("RIGHT")
        time.sleep(3)
        self.set_stop_effect()  # Stop turning before switching effects

        # Stop Effect
        self.set_stop_effect()
        time.sleep(3)
        
        self.update_strip([(0, 0, 0)] * self.num_leds)
        self.update_strip(led_array)
        self.get_logger().info("LED Test Sequence Complete")


    def set_leds(self, color):
        """Set all LEDs to a static color."""
        data = [self.encode_color(color) for _ in range(LED_COUNT)]
        data = [val for sublist in data for val in sublist]  # Flatten list
        self.spi.xfer2(data)

    
    def update_strip(self, led_array):
        """ Update LED strip with a color pattern based on motion """
        if len(led_array) != self.num_leds:
            self.get_logger().warn(f"LED array size mismatch: Expected {self.num_leds}, got {len(led_array)}")

        # Swap Red and Green for WS2812 GRB order
        corrected_leds = [(led[1], led[0], led[2]) for led in led_array]

        # Convert list of RGB tuples into a flat numpy array
        data = np.array(corrected_leds, dtype=np.uint8).flatten()

        #self.get_logger().info(f"LED Data Sent: {data.tolist()}")  # Debug Output

        # Send data to SPI using ws2812.write2812
        ws2812.write2812(self.spi, data)  # Ensure `ws2812` is imported


    def encode_color(self, color):
        """Convert an RGB color tuple to SPI-compatible format."""
        return [
            (color[1] >> 1) | 0x80,  # Green
            (color[0] >> 1) | 0x80,  # Red
            (color[2] >> 1) | 0x80,  # Blue
        ]

    def destroy_node(self):
        """Shutdown sequence and close SPI connection."""
        self.set_leds((0, 0, 0))  # Turn off LEDs
        self.spi.close()
        self.get_logger().info("LED Controller Node Stopped")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LEDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down LED Controller Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
