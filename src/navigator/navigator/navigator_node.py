import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')

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

        # Publisher for navigation decision
        self.safe_cmd_pub = self.create_publisher(Twist, 'safe_cmd_vel', 10)

        # Safety thresholds
        self.TOF_SAFETY_DISTANCE = 0.15  # 15 cm for ToF
        self.LIDAR_SAFETY_DISTANCE = 0.30  # 30 cm for Lidar
        self.safe_to_move = True  # Default: assume safe to move

        # Latest distance readings
        self.tof_distance = float('inf')
        self.lidar_distance = float('inf')

        self.get_logger().info("Navigator Node Initialized.")

    def tof_callback(self, msg):
        """Updates ToF sensor reading and filters invalid values."""
        if msg.data < 0:  # Ignore negative readings
            # self.get_logger().warn(f"⚠️ Invalid ToF reading: {msg.data:.3f}m (ignored)")
            return  # Skip processing this bad value

        self.tof_distance = msg.data
        self.evaluate_safety()


    def lidar_callback(self, msg):
        """Processes Lidar scan and updates safety check."""
        self.lidar_distance = min(msg.ranges)  # Get closest detected obstacle
        self.evaluate_safety()

    def evaluate_safety(self):
        """Determines if movement is safe based on sensor inputs."""
        if self.tof_distance < self.TOF_SAFETY_DISTANCE or self.lidar_distance < self.LIDAR_SAFETY_DISTANCE:
            self.safe_to_move = False
            self.get_logger().warn("🚨 Obstacle detected! Stopping movement.")
        else:
            self.safe_to_move = True

    def cmd_vel_callback(self, msg):
        """Applies safety rules to movement commands."""
        safe_cmd = Twist()

        if not self.safe_to_move:
            if msg.linear.x > 0 or abs(msg.linear.y) > 0.1:  # 🚨 Block forward & significant strafing
                safe_cmd.linear.x = 0.0   # Block forward movement
                safe_cmd.linear.y = 0.0   # Block strafing movement
                safe_cmd.angular.z = msg.angular.z  # Allow turning
                self.get_logger().warn("🚨 Movement blocked: Obstacle detected.")
            else:
                safe_cmd = msg  # Allow reverse and small adjustments
        else:
            safe_cmd = msg  # If safe, allow all movement

        self.safe_cmd_pub.publish(safe_cmd)



def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
