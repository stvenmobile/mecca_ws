import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String  # Needed for publishing velocity commands
from sensor_msgs.msg import JointState
import math

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        self.get_logger().info("Started node")
        # Subscribe to "safe" velocity commands
        self.subscription = self.create_subscription(
            Twist,
            'safe_cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.encoder_poll_timer = self.create_timer(0.2, self.poll_encoders)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_names = [
            'front_left_wheel_joint',
            'rear_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_right_wheel_joint'
        ]

        self.encoder_positions = [0, 0, 0, 0]  # ticks
        self.encoder_last_published = self.get_clock().now()

        self.create_subscription(
            String,
            '/serial_driver/output_data',
            self.serial_output_callback,
            10
        )

        # Publisher to send velocity commands to serial communication node
        self.serial_publisher = self.create_publisher(String, '/serial_driver/input', 10)

        # Speed scaling factors
        self.MAX_SPEED = 1100  # Maximum possible speed
        self.NORMAL_SCALE = 0.4  # Default scaling (40% of MAX_SPEED)
        self.TURBO_SCALE = 0.6   # Turbo mode scaling (60% of MAX_SPEED)
        
        self.NORMAL_ROTATE_SCALE = 1.4  # Boosted rotation scale
        self.TURBO_ROTATE_SCALE = 2.0   # 30% more for turbo rotation

        self.speed_scale = self.NORMAL_SCALE  # Default to normal mode

    def cmd_vel_callback(self, msg):
        """Convert velocity message into serial command and publish."""
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        scaled_x = int(linear_x * self.MAX_SPEED * self.speed_scale)
        scaled_y = int(linear_y * self.MAX_SPEED * self.speed_scale)

        if self.speed_scale == self.TURBO_SCALE:
            scaled_rot = int(angular_z * self.MAX_SPEED * self.TURBO_ROTATE_SCALE)
        else:
            scaled_rot = int(angular_z * self.MAX_SPEED * self.NORMAL_ROTATE_SCALE)

        # Construct command string
        command = f"V {scaled_x} {scaled_y} {scaled_rot}"

        # Publish to serial node
        msg_out = String()
        msg_out.data = command
        self.serial_publisher.publish(msg_out)

        # Log for debugging
        self.get_logger().debug(f"Published: {command}")

    def serial_output_callback(self, msg):
        if "Encoder Values" in msg.data:
            try:
                parts = msg.data.strip().split()
                m1 = int(parts[2].split('=')[1].strip(','))
                m2 = int(parts[3].split('=')[1].strip(','))
                m3 = int(parts[4].split('=')[1].strip(','))
                m4 = int(parts[5].split('=')[1])
                self.encoder_positions = [m1, m2, m3, m4]
                self.publish_joint_states()
                # self.get_logger().info(f"✅ Encoders: {self.encoder_positions}")
            except Exception as e:
                self.get_logger().warn(f"❌ Failed to parse encoder data: {e}")


    def publish_joint_states(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = self.joint_names

        # Convert encoder ticks to radians
        ticks_per_rev = 2420
        radians_per_tick = (2 * math.pi) / ticks_per_rev
        js.position = [ticks * radians_per_tick for ticks in self.encoder_positions]

        js.velocity = []
        js.effort = []
        self.joint_state_pub.publish(js)

    def poll_encoders(self):
        msg = String()
        msg.data = "I ENC"
        self.serial_publisher.publish(msg)
        # self.get_logger().info("🟢 Sent I ENC to STM32")


def main(args=None):
    print("🟢 Entered main() in mecca_driver_node")
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
