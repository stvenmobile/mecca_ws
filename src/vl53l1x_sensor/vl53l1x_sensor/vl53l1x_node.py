import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import VL53L1X  # Correct import for the sensor

class VL53L1XNode(Node):
    def __init__(self):
        super().__init__('vl53l1x_node')
        
        # Create a publisher for the distance readings
        self.publisher_ = self.create_publisher(Float32, 'tof_distance', 10)
        
        # Initialize VL53L1X sensor
        self.tof = VL53L1X.VL53L1X()
        self.tof.open()
        self.tof.start_ranging(2)  # Long-range mode

        # Set up a timer to publish readings at 10Hz
        self.timer = self.create_timer(0.1, self.publish_distance)
        self.get_logger().info("VL53L1X ToF sensor node started!")

    def publish_distance(self):
        """Reads distance from VL53L1X and publishes it."""
        distance_mm = self.tof.get_distance()
        distance_m = distance_mm / 1000.0  # Convert to meters

        msg = Float32()
        msg.data = distance_m
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published distance: {distance_m:.3f} meters")

    def stop_sensor(self):
        """Stops the sensor cleanly."""
        self.tof.stop_ranging()
        self.get_logger().info("VL53L1X sensor stopped.")

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
