import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')
        self.get_logger().info('NavNode for Mecca started.')

        # Publisher for commands
        self.cmd_pub = self.create_publisher(String, 'motor_command', 10)

        # Subscribe to Twist (from joystick, teleop_twist_joy, etc.)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x    # e.g. forward m/s
        linear_y = msg.linear.y    # e.g. left m/s
        angular_z = msg.angular.z  # e.g. rad/s

        # Convert floats to integer or scaled values suitable for your "V" command
        # E.g. multiply by 100 if that's your internal scale:
        vx = int(linear_x * 400)
        vy = int(linear_y * 400)
        wz = int(angular_z * 400)

        # Create the line: "V vx vy wz"
        command_str = f"V {vx} {vy} {wz} \r\n"

        # Log
        self.get_logger().info(
            f"Velocity command: {command_str}"
        )

        # Publish
        out_msg = String()
        out_msg.data = command_str
        self.cmd_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()