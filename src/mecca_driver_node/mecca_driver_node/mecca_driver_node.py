import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # Subscribe to the velocity commands (Twist messages)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Serial communication with STM32
        self.serial_port = serial.Serial('/dev/stm32_serial', 115200, timeout=0.1)

        # Speed scaling factors
        self.MAX_SPEED = 1200  # Maximum possible speed
        self.NORMAL_SCALE = 0.5  # Default scaling for linear movement (50% of MAX_SPEED)
        self.TURBO_SCALE = 0.7   # Turbo mode scaling (70% of MAX_SPEED)
        
        self.NORMAL_ROTATE_SCALE = 0.7  # 🚀 Boosted rotation scale
        self.TURBO_ROTATE_SCALE = 0.9   # 🚀 20% more for turbo rotation

        self.speed_scale = self.NORMAL_SCALE  # Default to normal mode

        self.get_logger().info("Motor driver node initialized.")

    def cmd_vel_callback(self, msg):
        """ Processes joystick velocity commands and sends motor control signals. """
        linear_x = msg.linear.x  # Forward/backward
        linear_y = msg.linear.y  # Left/right (strafing for Mecanum)
        angular_z = msg.angular.z  # Rotation

        # Scale linear movement normally
        scaled_x = int(linear_x * self.MAX_SPEED * self.speed_scale)
        scaled_y = int(linear_y * self.MAX_SPEED * self.speed_scale)

        # Scale rotation separately with higher boost
        if self.speed_scale == self.TURBO_SCALE:
            scaled_rot = int(angular_z * self.MAX_SPEED * self.TURBO_ROTATE_SCALE)
        else:
            scaled_rot = int(angular_z * self.MAX_SPEED * self.NORMAL_ROTATE_SCALE)

        # Send the scaled command via serial
        command = f"V {scaled_x} {scaled_y} {scaled_rot}\n"
        self.serial_port.write(command.encode())

        self.get_logger().info(f"Sent command: {command.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
