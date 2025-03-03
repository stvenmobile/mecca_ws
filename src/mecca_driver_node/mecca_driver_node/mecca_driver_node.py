import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan  # For obstacle detection
import serial

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

        # Serial communication with STM32
        self.serial_port = serial.Serial('/dev/stm32_serial', 115200, timeout=0.1)

        # Speed scaling factors
        self.MAX_SPEED = 1100  # Maximum possible speed
        self.NORMAL_SCALE = 0.4  # Default scaling (40% of MAX_SPEED)
        self.TURBO_SCALE = 0.6   # Turbo mode scaling (60% of MAX_SPEED)
        
        self.NORMAL_ROTATE_SCALE = 0.8  # Boosted rotation scale
        self.TURBO_ROTATE_SCALE = 0.9   # 30% more for turbo rotation

        self.speed_scale = self.NORMAL_SCALE  # Default to normal mode

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        scaled_x = int(linear_x * self.MAX_SPEED * self.speed_scale)
        scaled_y = int(linear_y * self.MAX_SPEED * self.speed_scale)

        if self.speed_scale == self.TURBO_SCALE:
            scaled_rot = int(angular_z * self.MAX_SPEED * self.TURBO_ROTATE_SCALE)
        else:
            scaled_rot = int(angular_z * self.MAX_SPEED * self.NORMAL_ROTATE_SCALE)

        # Send the final command to STM32
        command = f"V {scaled_x} {scaled_y} {scaled_rot}\n"
        self.serial_port.write(command.encode())

        # Reduce log frequency (only print 1 in every 20 commands)
        if hasattr(self, "log_counter"):
            self.log_counter += 1
        else:
            self.log_counter = 0

        if self.log_counter % 20 == 0:  # Only log every 10th message
            self.get_logger().info(f"Sent command: {command.strip()}")


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
