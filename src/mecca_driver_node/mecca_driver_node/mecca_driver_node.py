import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('mecca_driver_node')

        # Declare and retrieve ROS2 parameters
        self.serial_port_name = self.declare_parameter('serial_port', '/dev/stm32_serial').get_parameter_value().string_value
        self.baud_rate = self.declare_parameter('baud_rate', 115200).get_parameter_value().integer_value
        self.debug_serial = self.declare_parameter('serial_debug', True).get_parameter_value().bool_value
        
        # Subscribe to /cmd_vel topic for velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Subscribe to the output from serial_driver
        self.serial_output_sub = self.create_subscription(
            String,
            'serial_driver/output_data',
            self.serial_output_callback, 
            10
        )

        # Subscribe to motor_command topic for both motor and encoder requests
        self.motor_command_sub = self.create_subscription(
            String, 
            'motor_command',
            self.motor_command_callback, 
            10
        )
        
        # Publisher for encoder values
        self.encoder_publisher = self.create_publisher(String, 'encoder_values', 10)

        # Publisher to send commands to serial_driver
        self.serial_input_pub = self.create_publisher(String, 'serial_driver/input', 10)

    def motor_command_callback(self, msg):
        command = msg.data.strip()  # Clean up the incoming command
        # Check if it's an encoder request
        if command.startswith("I ENC"):
            self.request_encoder_values()
            return
        
        # Otherwise, treat it as a motor command
        self.send_command(command)

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # Convert the Twist message into a motor command
        # Example: Using a simple proportional conversion
        cmd_string = f"V {int(linear_x * 1000)} {int(linear_y * 1000)} {int(angular_z * 1000)}"
    
        self.send_command(cmd_string)
        if self.debug_serial:
            self.get_logger().info(f"Converted /cmd_vel to motor command: {cmd_string}")

    def send_command(self, cmd_string):
        cmd_string += "\r\n"  # Ensure command ends with CRLF
        msg = String()
        msg.data = cmd_string
        self.serial_input_pub.publish(msg)

        if self.debug_serial:
            self.get_logger().info(f"Sent: {cmd_string.strip()}")

    def serial_output_callback(self, msg):
        response = msg.data.strip()
        if response:
            if self.debug_serial:
                self.get_logger().info(f"Received: {response}")
            
            # Check if the response is an encoder value
            if response.startswith("ENC"):
                encoder_msg = String()
                encoder_msg.data = response
                self.encoder_publisher.publish(encoder_msg)
                if self.debug_serial:
                    self.get_logger().info(f"Published encoder values: {response}")

    def request_encoder_values(self):
        cmd_string = "I ENC"
        self.send_command(cmd_string)

    def close_connection(self):
        self.get_logger().info("Closed serial connection.")

def main(args=None):
    rclpy.init(args=args)

    mecca_driver = MotorDriverNode()

    try:
        rclpy.spin(mecca_driver)
    except KeyboardInterrupt:
        mecca_driver.get_logger().info("Shutting down motor driver node.")
    finally:
        mecca_driver.close_connection()
        mecca_driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
