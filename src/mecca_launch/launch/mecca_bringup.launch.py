from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    controllers_config = PathJoinSubstitution([
        FindPackageShare("mecca_driver_node"),
        "config",
        "controllers.yaml"
    ])

    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("mecca_description"),
                "urdf",
                "meccabot.xacro"
            ])
        ]),
        value_type=str
    )

    # 1. Robot State Publisher (Required for Controller Manager)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}]
    )

    # 2. Main ros2_control_node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content},
            controllers_config
        ],
        output="screen"
    )

    # 3. Mecca Driver Node (Your Python Logic)
    mecca_driver_node = Node(
        package='mecca_driver_node',
        executable='mecca_driver_node',  
        name='motor_driver_node',
        output='screen',
        remappings=[('safe_cmd_vel', '/cmd_vel')]
    )

    # 4. Serial Bridge Node - Fixed Parameter Types
    serial_bridge_node = Node(
        package='serial_driver',
        executable='serial_bridge',  
        name='serial_bridge',
        parameters=[{
            'device_name': '/dev/ttyUSB0',
            'baud_rate': 115200,      # Baud rate is usually an integer
            'flow_control': 'none',
            'parity': 'none',
            'stop_bits': '1',         # MUST BE STRING
            'data_bits': '8'          # MUST BE STRING
        }],
        output='screen'
    )

    # 5. Spawner for Mecanum Drive Controller
    mecanum_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["mecanum_drive_controller"]
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        mecca_driver_node,
        serial_bridge_node,
        mecanum_controller_spawner
    ])