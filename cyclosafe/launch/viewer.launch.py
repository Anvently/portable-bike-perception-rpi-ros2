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
            parameters=[{
                "topic_list": ["sonar1/range", "sonar2/range", "sonar3/range", "sonar4/range"]
            }],
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            output='screen',
            emulate_tty=True,
            parameters=[],
            arguments=[
                "images", "--ros-args", "--remap", "_image_transport:=compressed"
            ]
		),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0.01", "0.05", "0", "0", "0", "0", "laser", "sonar1/range"]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0.08", "0.05", "0", "0", "0", "0", "laser", "sonar2/range"]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0.155", "0.05", "0", "0", "0", "0", "laser", "sonar3/range"]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0.215", "0.05", "0", "0", "0", "0", "laser", "sonar4/range"]
        ),
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     output="screen" ,
        #     arguments=["0.", "0", "1", "0", "0", "0", "world", "laser"]
        # ),
    ])
    return ld

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
