import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String  # Needed for publishing velocity commands

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # Subscribe to "safe" velocity commands
        self.subscription = self.create_subscription(
            Twist,
            'safe_cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher to send velocity commands to serial communication node
        self.serial_publisher = self.create_publisher(String, '/serial_driver/input', 10)

        # Speed scaling factors
        self.MAX_SPEED = 1100  # Maximum possible speed
        self.NORMAL_SCALE = 0.4  # Default scaling (40% of MAX_SPEED)
        self.TURBO_SCALE = 0.6   # Turbo mode scaling (60% of MAX_SPEED)
        
        self.NORMAL_ROTATE_SCALE = 0.8  # Boosted rotation scale
        self.TURBO_ROTATE_SCALE = 0.9   # 30% more for turbo rotation

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
        self.get_logger().info(f"Published: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
