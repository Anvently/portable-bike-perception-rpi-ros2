from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cyclosafe',
            executable='camera_pi',
            namespace='camera',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'queue_size': '40',
                 'resolution': [1200, 800],
                 'interval': 0.25,
                 'compression': 95,
                 'preview': True}
            ]
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            namespace='lidar360',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'serial_port': '/dev/ttyACM0',
                 'serial_baudrate': 1000000,
                 'topic_name': 'laser_scan',
                 'scan_frequency': 50.0,
                 'scan_mode': ''}
            ]
        ),
        Node(
            package='cyclosafe',
            executable='gps',
            namespace='gps',
            output='screen',
            emulate_tty=True,
            parameters=[
                {}
            ]
		),
    ])
