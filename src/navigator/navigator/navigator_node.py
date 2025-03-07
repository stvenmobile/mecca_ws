import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')

        # Subscribe to ToF sensor
        self.tof_subscription = self.create_subscription(
            Float32,
            'tof_distance',
            self.tof_callback,
            10
        )

        # Subscribe to Lidar scan
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10
        )

        # Subscribe to joystick commands
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            'cmd_vel',  # Raw joystick input
            self.cmd_vel_callback,
            10
        )

        # Publisher for safe movement commands
        self.safe_cmd_pub = self.create_publisher(Twist, 'safe_cmd_vel', 10)

        # Publisher for LED commands
        self.led_cmd_pub = self.create_publisher(String, 'led_commands', 10)

        # Publisher for motor commands (to request encoder data)
        self.motor_command_pub = self.create_publisher(String, '/motor_command', 10)

        # Subscribe to listen for general status requests
        self.info_request_sub = self.create_subscription(
            String,
            '/mecca_info',  # General info request topic
            self.mecca_info_callback,
            10
        )

        # Subscribe to serial output (to process I ENC responses)
        self.serial_output_sub = self.create_subscription(
            String,
            '/serial_driver/output_data',
            self.serial_output_callback,
            10
        )

        # Publisher for responding to /mecca_info requests
        self.info_response_pub = self.create_publisher(String, '/mecca_info_response', 10)

        # Safety thresholds
        self.TOF_SAFETY_DISTANCE = 0.15  # 15 cm for ToF
        self.LIDAR_SAFETY_DISTANCE = 0.30  # 30 cm for Lidar
        self.safe_to_move = True  # Default: assume safe to move

        # Latest sensor values
        self.tof_distance = float('inf')
        self.lidar_distance = float('inf')
        self.battery_voltage = 0.0  # Placeholder (update if you have battery monitoring)

        self.get_logger().info("Navigator Node Initialized.")

    def mecca_info_callback(self, msg):
        """Handles general info requests from /mecca_info."""
        request = msg.data.lower()

        if request == "get_encoders":
            enc_msg = String()
            enc_msg.data = "I ENC"
            self.motor_command_pub.publish(enc_msg)
            self.get_logger().info("Requested Encoder Data: I ENC")

        elif request == "get_battery":
            # Simulate battery voltage retrieval (replace with actual method)
            battery_status = f"Battery Voltage: {self.battery_voltage:.2f}V"
            response_msg = String()
            response_msg.data = battery_status
            self.info_response_pub.publish(response_msg)
            self.get_logger().info(f"Published Battery Status: {battery_status}")

        elif request == "get_lidar_scan":
            # Simulate sending last known Lidar distance
            lidar_status = f"Lidar Closest Object: {self.lidar_distance:.2f}m"
            response_msg = String()
            response_msg.data = lidar_status
            self.info_response_pub.publish(response_msg)
            self.get_logger().info(f"Published Lidar Data: {lidar_status}")

        else:
            self.get_logger().warn(f"Unknown /mecca_info request: {request}")

    def serial_output_callback(self, msg):
        """Processes responses from /serial_driver/output_data."""
        if "I ENC" in msg.data:
            self.get_logger().info(f"Encoder Data Received: {msg.data}")
            # Optionally, parse and store encoder values
            response_msg = String()
            response_msg.data = msg.data
            self.info_response_pub.publish(response_msg)

    def tof_callback(self, msg):
        """Updates ToF sensor reading and filters invalid values."""
        if msg.data < 0:  # Ignore negative readings
            return  # Skip processing this bad value

        self.tof_distance = msg.data
        self.evaluate_safety()

    def lidar_callback(self, msg):
        """Processes Lidar scan and updates safety check."""
        self.lidar_distance = min(msg.ranges)  # Get closest detected obstacle
        self.evaluate_safety()

    def evaluate_safety(self):
        """Determines if movement is safe based on sensor inputs."""
        if self.tof_distance < self.TOF_SAFETY_DISTANCE or self.lidar_distance < self.LIDAR_SAFETY_DISTANCE:
            self.safe_to_move = False
            self.get_logger().warn("🚨 Obstacle detected! Stopping movement.")
        else:
            self.safe_to_move = True

    def cmd_vel_callback(self, msg):
        """Applies safety rules to movement commands and sends LED updates."""
        safe_cmd = Twist()

        if not self.safe_to_move:
            if msg.linear.x > 0 or abs(msg.linear.y) > 0.1:  # 🚨 Block forward & significant strafing
                safe_cmd.linear.x = 0.0   # Block forward movement
                safe_cmd.linear.y = 0.0   # Block strafing movement
                safe_cmd.angular.z = msg.angular.z  # Allow turning
                self.get_logger().warn("🚨 Movement blocked: Obstacle detected.")
            else:
                safe_cmd = msg  # Allow reverse and small adjustments
        else:
            safe_cmd = msg  # If safe, allow all movement

        # Publish the "safe" movement command
        self.safe_cmd_pub.publish(safe_cmd)

        led_msg = String()
        if safe_cmd.linear.x > 0:  # Forward
            led_msg.data = "fwd"
        elif safe_cmd.linear.x < 0:  # Backward
            led_msg.data = "bwd"
        elif safe_cmd.angular.z > 0:  # Left Turn
            led_msg.data = "left"
        elif safe_cmd.angular.z < 0:  # Right Turn 
            led_msg.data = "right"
        else:  # Stopped
            led_msg.data = "stop"

        # Publish LED command
        self.led_cmd_pub.publish(led_msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
