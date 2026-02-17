import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String

class SerialTranslator(Node):
    def __init__(self):
        super().__init__('serial_translator')
        self.sub = self.create_subscription(UInt8MultiArray, '/serial_read', self.callback, 10)
        self.pub = self.create_publisher(String, '/serial_text', 10)
        
        # New: Internal buffer to hold partial strings
        self.msg_buffer = ""
        self.get_logger().info("Serial Translator (Buffered) Started.")

    def callback(self, msg):
        # 1. Append new bytes to buffer
        new_chars = "".join(chr(b) for b in msg.data)
        self.msg_buffer += new_chars
        
        # 2. Process all complete lines (ending in \n)
        while "\n" in self.msg_buffer:
            # Split the first complete line from the rest
            line, self.msg_buffer = self.msg_buffer.split("\n", 1)
            
            # 3. Publish the cleaned line
            out_msg = String()
            out_msg.data = line.strip()
            self.pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialTranslator()
    rclpy.spin(node)
    node.destroy_node(); rclpy.shutdown()