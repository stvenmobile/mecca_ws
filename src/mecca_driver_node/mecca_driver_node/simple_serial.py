import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SimpleSerialNode(Node):
    def __init__(self):
        super().__init__('simple_serial_node')

        # Serial port configuration
        self.port = '/dev/stm32_serial'
        self.baudrate = 115200

        # Attempt to establish serial connection
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"Connected to {self.port} at {self.baudrate} baud.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to {self.port}: {e}")
            self.serial_conn = None

        # ROS2 Publisher to publish incoming data from STM32
        self.publisher = self.create_publisher(String, '/serial_driver/output_data', 10)

        # ROS2 Subscriber to receive velocity commands and send over serial
        self.subscription = self.create_subscription(
            String,
            '/serial_driver/input',
            self.send_serial_data,
            10
        )

        # Timer to periodically read from serial port
        self.timer = self.create_timer(0.1, self.read_serial_data)  # 10 Hz

    def send_serial_data(self, msg):
        """Send data to the serial device."""
        if self.serial_conn and self.serial_conn.is_open:
            cmd = (msg.data + '\r\n').encode("ascii")  # Ensure proper encoding
            # self.get_logger().info(f"DEBUG: Sending -> {cmd}")  # Log raw bytes
            self.serial_conn.write(cmd)
            self.serial_conn.flush()
            # self.get_logger().info(f"Sent: {msg.data}")
        else:
            self.get_logger().error("Serial connection is not open.")

    def read_serial_data(self):
        """Read data from the serial device."""
        if self.serial_conn and self.serial_conn.is_open:
            if self.serial_conn.in_waiting > 0:
                raw_data = self.serial_conn.readline()
                # self.get_logger().info(f"DEBUG: Raw RX Bytes -> {raw_data}")  # Log raw bytes
                
                try:
                    incoming_data = raw_data.decode().strip()
                    # self.get_logger().info(f"DEBUG: Decoded RX -> {incoming_data}")
                except UnicodeDecodeError as e:
                    self.get_logger().error(f"Unicode Decode Error: {e}")
                    return

                if incoming_data:
                    ros_msg = String()
                    ros_msg.data = incoming_data
                    self.publisher.publish(ros_msg)
                    # self.get_logger().info(f"Published: {ros_msg.data}")
        else:
            self.get_logger().error("Serial connection is not open.")


    def destroy_node(self):
        """Close serial connection when shutting down."""
        if self.serial_conn and self.serial_conn.is_open:
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
