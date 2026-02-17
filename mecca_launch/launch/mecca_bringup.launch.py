from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
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
            ])
        ]),
        value_type=str
    )

    # 3. Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}]
    )

    # 4. Main ros2_control_node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content},
            controllers_config
        ],
        output="screen"
    )

    # 5. Spawner for Mecanum Drive Controller
    # Remaps the stamped 'reference' topic to standard '/cmd_vel'
    mecanum_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "mecanum_drive_controller",
                    "--controller-ros-args=-p use_stamped_vel:=false", # Single string, no spaces between flag and value
                ],
                remappings=[
                    ('/mecanum_drive_controller/reference', '/cmd_vel'),
                ]
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
                arguments=["joint_broad"]
            )
        ]
    )

    # 7. LED Controller Node
    # Starts the WS2812 animation controller
    led_controller_node = TimerAction(
        period=4.0, # Starts slightly after the joint broadcaster
        actions=[
            Node(
                package="mecca_driver_node",
                executable="led_controller_node",
                name="led_controller",
                output="screen"
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        mecanum_controller_spawner,
        joint_broadcaster_spawner,
        led_controller_node # 🏁 Added to the list
    ])
