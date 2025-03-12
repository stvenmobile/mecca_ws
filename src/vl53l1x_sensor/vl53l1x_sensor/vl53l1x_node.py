import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import VL53L1X  # Correct import for the sensor

class VL53L1XNode(Node):
    def __init__(self):
        super().__init__('vl53l1x_node')
        time.sleep(0.5)  # Small delay to allow sensor to stabilize
        
        # Create a publisher for the distance readings
        self.publisher_ = self.create_publisher(Float32, 'tof_distance', 10)
        
        # Initialize VL53L1X sensor
        self.tof = VL53L1X.VL53L1X()
        self.tof.open()
        self.tof.start_ranging(2)  # Long-range mode
        self.tof.set_timing_budget(15)  # Reduce to 20ms (default is ~33ms)
        self.tof.set_inter_measurement_period(20)  # Lower to 20ms for faster updates

        # Averaging buffer (last 5 readings)
        self.distances = []

        # Set up a timer to collect readings more frequently 50ms (20Hz)
        self.collect_timer = self.create_timer(0.03, self.collect_distance)

        # Separate timer to publish the averaged reading 100ms (10Hz)
        self.publish_timer = self.create_timer(0.066, self.publish_distance)

        self.get_logger().info("✅ VL53L1X ToF sensor node started!")

    def collect_distance(self):
        """Collects valid distance readings for averaging."""
        distance_mm = self.tof.get_distance()
        distance_m = distance_mm / 1000.0  # Convert to meters

        #if distance_m > 0:  # Ignore negative or zero readings
        #    self.distances.append(distance_m)
        #    if len(self.distances) > 5:  # Keep only the last 5 readings
        #        self.distances.pop(0)
        #else:
        #    self.get_logger().warn(f"⚠️ Ignored invalid distance reading: {distance_m:.3f}m")

    def publish_distance(self):
        """Reads distance from VL53L1X and publishes it."""
        distance_mm = self.tof.get_distance()

        # Convert to meters
        distance_m = distance_mm / 1000.0  

        # 🛑 LOG IF SENSOR RETURNS INVALID DATA
        if distance_m <= 0 or distance_m > 4.0:  # Max valid range ~4m
            self.get_logger().warn(f"⚠️ Ignored invalid distance reading: {distance_m:.3f}m")
            return  

        msg = Float32()
        msg.data = distance_m
        self.publisher_.publish(msg)
        self.last_log_time = time.time()
        if time.time() - self.last_log_time > 1.0:  # Only log once per second
            self.get_logger().info(f"📡 Published Avg ToF Distance: {distance_m:.3f}m")
            self.last_log_time = time.time()

    def stop_sensor(self):
        """Stops the sensor cleanly."""
        self.tof.stop_ranging()
        self.get_logger().info("🛑 VL53L1X sensor stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = VL53L1XNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_sensor()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
