from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 0. Launch arguments
    with_joy_arg = DeclareLaunchArgument(
        'with_joy',
        default_value='true',
        description='Start joy_node + teleop_twist_joy (set false when using drive.py)'
    )
    with_joy = LaunchConfiguration('with_joy')

    hw_debug_arg = DeclareLaunchArgument(
        'hw_debug',
        default_value='false',
        description='Enable verbose write() logging in the hardware interface (true/false)'
    )
    hw_debug = LaunchConfiguration('hw_debug')

    with_led_arg = DeclareLaunchArgument(
        'with_led',
        default_value='true',
        description='Start the WS2812 LED controller (set false to disable)'
    )
    with_led = LaunchConfiguration('with_led')

    # 1. Path to your unified controllers.yaml
    controllers_config = PathJoinSubstitution([
        FindPackageShare("mecca_driver_node"),
        "config",
        "controllers.yaml"
    ])

    # 2. Generate robot_description from xacro
    # We wrap the Command in ParameterValue to force it to be treated as a raw string
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([
                FindPackageShare("mecca_description"),
                "urdf",
                "meccabot.xacro"
            ]),
            " hw_debug:=", hw_debug
        ]),
        value_type=str
    )

    # 3. Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description_content,
            "use_sim_time": False  # Add this explicitly
        }]
    )

    # 4. Main ros2_control_node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": False}, # Add this explicitly
            controllers_config
        ],
        output="screen"
    )
    

    # 5. Spawner for Mecanum Drive Controller
    mecanum_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["mecanum_drive_controller", "--unload-on-kill"],
            )
        ]
    )

    # 6. Spawner for Joint State Broadcaster
    joint_broadcaster_spawner = TimerAction(
        period=3.5,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_broad", "--unload-on-kill"],
            )
        ]
    )

    # 7. LED Controller Node
    # Starts the WS2812 animation controller
    led_controller_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="mecca_driver_node",
                executable="led_controller_node",
                name="led_controller",
                output="screen",
                condition=IfCondition(with_led)
            )
        ]
    )

    # 8. Add the path to your joy_teleop config
    joy_teleop_config = PathJoinSubstitution([
        FindPackageShare("mecca_launch"), # Move it here for best practice
        "config",
        "joy_teleop.yaml"
    ])

    # 9. Revised Joy Node for Jazzy
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,             # Standard for /dev/input/js0
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,    # Critical for continuous cmd_vel
        }],
        condition=IfCondition(with_joy)
    )

    # 10. Teleop Node - FORCED TO STAMPED
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[
            joy_teleop_config,
            {'publish_stamped_twist': True} # This is the magic switch
        ],
        remappings=[('/cmd_vel', '/mecanum_drive_controller/reference')],
        condition=IfCondition(with_joy)
    )

    return LaunchDescription([
        with_joy_arg,
        hw_debug_arg,
        with_led_arg,
        robot_state_publisher_node,
        ros2_control_node,
        mecanum_controller_spawner,
        joint_broadcaster_spawner,
        led_controller_node,
        joy_node,
        teleop_node
    ])
