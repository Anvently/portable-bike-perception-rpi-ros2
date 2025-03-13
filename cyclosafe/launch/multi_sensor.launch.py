from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cyclosafe',
            executable='sensor',
            namespace='sensor1',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'baud': '57600',
                 'port': '/dev/ttyUSB0'}
            ]
        ),
        Node(
            package='cyclosafe',
            executable='sensor',
            namespace='sensor2',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'baud': '57600',
                 'port': '/dev/ttyUSB1'}
            ]
        )
    ])