from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gros-pote',
            namespace='gros_pote',
            executable='gros-pote-node',
            name='robot',
            parameters=[
                {"enable_joypad": 0},
                {"UART_address": "/dev/ttyUSB0"}
            ]
        )
    ])