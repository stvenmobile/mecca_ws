from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    with_led_arg = DeclareLaunchArgument(
        'with_led',
        default_value='true',
        description='Start the WS2812 LED controller (set false to disable)'
    )
    with_led = LaunchConfiguration('with_led')

    joy_teleop_config = PathJoinSubstitution([
        FindPackageShare('mecca_launch'),
        'config',
        'joy_teleop.yaml'
    ])

    # 1. Serial bridge — direct path to STM32, no ros2_control involvement
    simple_serial_node = Node(
        package='mecca_driver_node',
        executable='simple_serial_node',
        name='simple_serial_node',
        output='screen',
    )

    # 2. Joystick reader
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }]
    )

    # 3. Twist publisher from joystick axes/buttons → /cmd_vel (plain Twist)
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[
            joy_teleop_config,
            {'publish_stamped_twist': False}   # plain Twist to /cmd_vel
        ],
    )

    # 4. Bridge: /cmd_vel (Twist) → V mm/s commands → /serial_driver/input
    #    Also publishes LED state changes to /led_commands
    teleop_serial_bridge_node = Node(
        package='mecca_driver_node',
        executable='teleop_serial_bridge_node',
        name='teleop_serial_bridge',
        output='screen',
    )

    # 5. LED controller (optional)
    led_controller_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='mecca_driver_node',
                executable='led_controller_node',
                name='led_controller',
                output='screen',
                condition=IfCondition(with_led)
            )
        ]
    )

    return LaunchDescription([
        with_led_arg,
        simple_serial_node,
        joy_node,
        teleop_node,
        teleop_serial_bridge_node,
        led_controller_node,
    ])
