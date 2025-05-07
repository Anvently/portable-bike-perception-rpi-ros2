from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration
# from rclpy.time import Time
# from rclpy.clock import Clock
import os, sys
from ament_index_python.packages import get_package_share_directory

package_dir = get_package_share_directory('cyclosafe')
launch_dir = os.path.join(package_dir, 'launch')
print(launch_dir)
sys.path.insert(0, launch_dir)
from config import sensors_list, SensorTypeEnum


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
            'launch',
            'view.rviz')
    ld = []
    if bag != "":
        ld.extend([
            Node(
                package="cyclosafe_player",
                executable="player",
                name="player",
                output='screen',
                arguments=[bag],
                emulate_tty=True,
                additional_env={'QT_QPA_PLATFORM': 'xcb'}
            )
        ])
    else:
        ld.extend([Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            output='screen',
            emulate_tty=True,
            parameters=[],
            arguments=[
                "images", "--ros-args", "--remap", "_image_transport:=compressed"
            ]
		)])
    ld.extend(
        [Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        )]),
    # Range circle visualizer
    ld.extend([
        Node(
            package='cyclosafe',
            executable='range_circle_transform',
            output='screen',
            emulate_tty=True,
            parameters=[{
                "topic_list": [sensor.topic for sensor in sensors_list if sensor.type == SensorTypeEnum.RangeSensor],
                "colors": [sensor.get_color_int32() for sensor in sensors_list if sensor.type == SensorTypeEnum.RangeSensor]
            }],
        ),
    ])
    # Frame transformations
    for sensor in sensors_list:
        if sensor.transform and len(sensor.transform) > 0:
            ld.extend([
                    Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    output="screen" ,
                    arguments=sensor.transform
                )])
    # Board to world transformation
    ld.extend([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["--x", "0.0", "--y", "0.0", "--z", "0.85", "--roll", "0", "--pitch", "0.0", "--yaw", "0", "--frame-id", "world", "--child-frame-id", "board"],
        ),
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     output="screen" ,
        #     arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0", "--roll", "0", "--pitch", "-1.57", "--yaw", "0", "--frame-id", "world_on_x", "--child-frame-id", "world"],
        # ),
    ])
    return ld

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    qt_env = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')
    ld.add_action(qt_env)
    ld.add_action(opfunc)
    return ld
