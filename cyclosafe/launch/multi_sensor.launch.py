from launch import LaunchDescription
from launch_ros.actions import Node
# from rclpy.time import Time
# from rclpy.clock import Clock
import time

def generate_launch_description():
    time_start = time.time()
    print(f"python start time = {time_start}")
    return LaunchDescription([
        Node(
            package='cyclosafe_hub',
            executable='hub',
            namespace='',
            output='screen',
            emulate_tty=True,
            parameters=[
                { 'start_time': float(time_start)}
            ]
        ),
        Node(
            package='cyclosafe',
            executable='gps',
            namespace='',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'baud': 115200,
                 'port': '/dev/ttyACM0',
                 'start_time': float(time_start)}
            ]
        ),
        Node(
            package='cyclosafe',
            executable='camera_webcam',
            namespace='',
            output='screen',
            emulate_tty=True,
            parameters=[
               {'queue_size': 40,
                 'resolution': [1200, 800],
                 'interval': 0.25,
                 'compression': 95,
                 'preview': True,
                 'start_time': float(time_start)}
            ]
        )
    ])