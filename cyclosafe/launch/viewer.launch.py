from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration
# from rclpy.time import Time
# from rclpy.clock import Clock
import datetime, time
import os
from ament_index_python.packages import get_package_share_directory

def str2bool(v):
    return v.lower() in ("yes", "true", "t", "1")

launch_args = [
    DeclareLaunchArgument('bag', default_value=TextSubstitution(text=""), description="Specify a bag from which the data will be played"),
]

def setup_directory(parent_dir: str, time_start: float) -> str:
    pass

def launch_setup(context):
    bag = LaunchConfiguration('bag').perform(context)
    rviz_config_path = os.path.join(
            get_package_share_directory('cyclosafe'),
            'view.rviz')
    ld = []
    if bag != "":
        ld.extend([
            ExecuteProcess(
                cmd=['gnome-terminal', '--', 'ros2', 'bag', 'play', bag],
                output='screen',
            )
        ])
    ld.extend(
        [Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),
        Node(
            package='cyclosafe',
            executable='range_circle_transform',
            output='screen',
            emulate_tty=True,
            parameters=[],
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            output='screen',
            emulate_tty=True,
            parameters=[{
                "image_transport": "compressed",
                "topic": "images"
            }]
		),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "90", "0", "0", "sonar", "laser"]
        ),
    ])
    return ld

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
