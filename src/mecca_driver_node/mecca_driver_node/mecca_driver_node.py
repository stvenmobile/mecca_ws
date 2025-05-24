import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String  # Needed for publishing velocity commands
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import math
import time


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
        
        # Timer for encoder polling
        self.encoder_poll_timer = self.create_timer(0.2, self.poll_encoders)
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # TF broadcaster for robot pose
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Joint names (must match your URDF exactly)
        self.joint_names = [
            'front_left_wheel_joint',
            'rear_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_right_wheel_joint'
        ]

        # Encoder and odometry data
        self.encoder_positions = [0, 0, 0, 0]  # ticks
        self.previous_encoder_positions = [0, 0, 0, 0]  # for velocity calculation
        self.encoder_last_published = self.get_clock().now()
        
        # Robot pose tracking for odometry
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.last_time = self.get_clock().now()
        
        # Robot physical parameters (adjust to match your robot)
        self.wheel_radius = 0.050  # 50mm wheel radius
        self.wheel_base = 0.175    # 175mm wheelbase (front to rear)
        self.track_width = 0.175   # 175mm track width (left to right)
        
        # Encoder parameters
        self.ticks_per_rev = 2420
        self.radians_per_tick = (2 * math.pi) / self.ticks_per_rev
        
        # Serial communication
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
        self.TURBO_ROTATE_SCALE = 2.0
