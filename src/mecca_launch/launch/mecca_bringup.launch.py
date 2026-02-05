from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Generate robot_description from xacro
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
        value_type=str  # 🛠️ Ensures it's treated as a string param
    )

    # Load robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}]
    )

    # Load ros2_control_node with its parameter file
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("mecca_driver_node"),
                "config",
                "ros2_control_node.yaml"
            ])
        ]
    )

    # Delay the controller spawners until ros2_control_node is ready
    mecanum_controller_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "mecanum_drive_controller",
                ],
                parameters=[
                    PathJoinSubstitution([
                        FindPackageShare("mecca_driver_node"),
                        "config",
                        "mecanum_drive_controller.yaml"
                    ])
                ]
            )
        ]
    )

    joint_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_broad"]
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        ros2_control_node,
        mecanum_controller_spawner,
        joint_broadcaster_spawner
    ])
