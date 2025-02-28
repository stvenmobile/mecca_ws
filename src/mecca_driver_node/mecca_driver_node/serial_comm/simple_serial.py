import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SimpleSerialNode(Node):

    def __init__(self):
        super().__init__('simple_serial_node')

        # Serial port configuration
        self.port = '/dev/ttyUSB0'   # Adjust as needed
        self.baudrate = 115200       # Confirm with Yahboom controller settings

        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"Connected to {self.port} at {self.baudrate} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to {self.port}: {e}")

        # ROS2 Subscriber to send commands to the serial device
        self.subscription = self.create_subscription(
            String,
            'serial_write',
            self.send_serial_data,
            10
        )

        # ROS2 Publisher to read incoming data
        self.publisher = self.create_publisher(String, 'serial_read', 10)

        # Timer to periodically read from serial port
        self.timer = self.create_timer(0.1, self.read_serial_data)  # 10 Hz

    def send_serial_data(self, msg):
        """Send data to the serial device."""
        if self.serial_conn.is_open:
            self.serial_conn.write((msg.data + '\n').encode())
            self.get_logger().info(f"Sent: {msg.data}")
        else:
            self.get_logger().error("Serial connection is not open.")

    def read_serial_data(self):
        """Read data from the serial device."""
        if self.serial_conn.is_open:
            if self.serial_conn.in_waiting > 0:
                incoming_data = self.serial_conn.readline().decode().strip()
                if incoming_data:
                    self.get_logger().info(f"Received: {incoming_data}")
                    ros_msg = String()
                    ros_msg.data = incoming_data
                    self.publisher.publish(ros_msg)
        else:
            self.get_logger().error("Serial connection is not open.")

    def destroy_node(self):
        """Close serial connection when shutting down."""
        if self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info("Serial connection closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
