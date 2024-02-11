from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enable_joypad', default_value='0'),
        DeclareLaunchArgument('UART_address', default_value='/dev/ttyUSB0'),
        Node(
            package='gros-pote',
            namespace='gros_pote',
            executable='gros-pote-node',
            name='robot',
            parameters=[
                {"enable_joypad": LaunchConfiguration('enable_joypad')},
                {"UART_address": LaunchConfiguration('UART_address')}
            ]
        )
    ])