from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import LifecycleNode, Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # --- 1. Path Resolutions ---
    controllers_config = PathJoinSubstitution([
        FindPackageShare("mecca_driver_node"),
        "config",
        "controllers.yaml"
    ])

    serial_config = PathJoinSubstitution([
        FindPackageShare("mecca_driver_node"),
        "config",
        "serial_bridge.yaml"
    ])

    # --- 2. URDF Processing ---
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

    # --- 3. Core Nodes ---
    
    # Publishes transforms and silences the root link inertia warning
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description_content,
            "ignore_root_link_inertia": True
        }]
    )

    # Manages the hardware interface and controller loading
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description_content}, controllers_config],
        output="screen"
    )

    # The Serial Bridge (Lifecycle-ready, activated in startup script)
    serial_bridge_node = LifecycleNode(
        package='serial_driver',
        executable='serial_bridge',  
        name='serial_bridge',
        namespace='',
        parameters=[serial_config],
        output="screen"
    )

    # Serial translator to publish human-readable data
    serial_translator_node = Node(
        package='mecca_driver_node',
        executable='serial_translator',
        name='serial_translator',
        output='screen'
    )

    # Custom Driver Logic for Byte-Mode Parsing
    mecca_driver_node = Node(
        package='mecca_driver_node',
        executable='mecca_driver_node',  
        name='motor_driver_node',
        output='screen'
    )

    # --- 4. Delayed Spawner ---
    # We keep a 6-second delay here to ensure the shell script has finished 
    # the 'configure' and 'activate' steps before the controller starts up.
    mecanum_controller_spawner = TimerAction(
        period=6.0,
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
        serial_translator_node,
        mecanum_controller_spawner
    ])