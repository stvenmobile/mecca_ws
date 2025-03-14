import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')
        
        self.safe_cmd_pub = self.create_publisher(Twist, 'safe_cmd_vel', 10)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        self.safe_to_move = True  # Updated dynamically based on sensors
        self.last_safe_cmd = Twist()  # Track the last sent command
        self.stop_command_counter = 0  # Counter for reducing redundant stops
        self.stop_publish_interval = 10  # Send stop command once every N cycles (adjustable)

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
        self.TOF_SAFETY_DISTANCE = 0.15     # 15cm 
        self.LIDAR_SAFETY_DISTANCE = 0.15   # 15cm 
        self.safe_to_move = True            # Default: assume safe to move
        self.stopped_for_obstacle = False   # Track if robot stopped for an obstacle


        # Latest sensor values
        self.tof_distance = float('inf')
        self.lidar_distance = float('inf')
        self.battery_voltage = 0.0  # Placeholder (update if you have battery monitoring)

        self.get_logger().info("Navigator Node Initialized.")

    def evaluate_safety(self):
        """Determines if movement is safe based on sensor inputs."""
        if self.tof_distance < self.TOF_SAFETY_DISTANCE or self.lidar_distance < self.LIDAR_SAFETY_DISTANCE:
            if not self.stopped_for_obstacle:
                self.safe_to_move = False
                self.stopped_for_obstacle = True  # Mark as stopped
                self.get_logger().warn("🚨 Obstacle detected! Stopping movement.")
        else:
            self.safe_to_move = True
            self.stopped_for_obstacle = False  # Reset once clear

    
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
            self.get_logger().debug(f"Encoder Data Received: {msg.data}")
            # Optionally, parse and store encoder values
            response_msg = String()
            response_msg.data = msg.data
            self.info_response_pub.publish(response_msg)

    def tof_callback(self, msg):
        """Updates ToF sensor reading and filters invalid values."""
        if msg.data > 0:  # Only accept valid values
            self.tof_distance = msg.data
            self.evaluate_safety()  # Re-evaluate after every valid reading

    def lidar_callback(self, msg):
        """Processes Lidar scan and updates safety check."""
        self.lidar_distance = min(msg.ranges)  # Get closest detected obstacle
        self.evaluate_safety()

    def evaluate_safety(self):
        """Determines if movement is safe and stops immediately if needed."""
        if self.tof_distance < self.TOF_SAFETY_DISTANCE or self.lidar_distance < self.LIDAR_SAFETY_DISTANCE:
            if not self.stopped_for_obstacle:  # Avoid redundant stop commands
                self.safe_to_move = False
                self.stopped_for_obstacle = True
                self.get_logger().warn("🚨 EMERGENCY STOP! Object detected.")

                # 🚨 Publish stop command only once
                stop_cmd = Twist()
                self.safe_cmd_pub.publish(stop_cmd)
        else:
            self.safe_to_move = True
            self.stopped_for_obstacle = False  # Reset state


    def cmd_vel_callback(self, msg):
        """Applies safety rules to movement commands and sends LED updates."""
        safe_cmd = Twist()
        led_msg = String()  # Add LED message object

        if not self.safe_to_move:
            if msg.linear.x > 0:  # 🚨 Block only forward movement
                safe_cmd.linear.x = 0.0   # Block forward movement
                safe_cmd.linear.y = msg.linear.y  # Allow strafing
                safe_cmd.angular.z = msg.angular.z  # Allow turning
                self.get_logger().warn("🚨 Forward movement blocked: Obstacle detected.")
                led_msg.data = "stop"  # Indicate obstacle
            else:
                safe_cmd = msg  # Allow all other movements, including reverse + turning
                led_msg.data = "bwd" if msg.linear.x < 0 else "idle"
        else:
            safe_cmd = msg  # If safe, allow all movement
            # **Determine LED state**
            if safe_cmd.linear.x > 0:
                led_msg.data = "fwd"
            elif safe_cmd.linear.x < 0:
                led_msg.data = "bwd"
            elif safe_cmd.linear.y > 0:
                led_msg.data = "right"
            elif safe_cmd.linear.y < 0:
                led_msg.data = "left"
            elif safe_cmd.angular.z > 0:
                led_msg.data = "left"
            elif safe_cmd.angular.z < 0: 
                led_msg.data = "right"
            else:
                led_msg.data = "stop"  # Robot is idle

        # **Ensure stop command is published when joystick is centered**
        if (
            safe_cmd.linear.x == 0.0
            and safe_cmd.linear.y == 0.0
            and safe_cmd.angular.z == 0.0
            ):
            # **Check if the last command was already stop**
            if (
                self.last_safe_cmd.linear.x == 0.0
                and self.last_safe_cmd.linear.y == 0.0
                and self.last_safe_cmd.angular.z == 0.0
                ):
                # **Send stop only once every N cycles**
                self.stop_command_counter += 1
                if self.stop_command_counter < self.stop_publish_interval:
                    self.led_cmd_pub.publish(led_msg)  # Ensure LED still updates
                    return  # Skip sending redundant stop
                self.stop_command_counter = 0  # Reset counter
                #self.get_logger().info("🔴 Joystick idle, sending periodic stop command.")
            #else:
                #self.get_logger().info("🔴 Joystick idle, sending immediate stop command.")

        # **Publish the "safe" movement command**
        self.safe_cmd_pub.publish(safe_cmd)
        self.last_safe_cmd = safe_cmd  # Update last sent command

        # **Ensure LED is updated after movement command**
        self.led_cmd_pub.publish(led_msg)  # This now always gets called


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
