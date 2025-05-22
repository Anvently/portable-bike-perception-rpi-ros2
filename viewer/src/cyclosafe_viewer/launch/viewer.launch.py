from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration
# from rclpy.time import Time
# from rclpy.clock import Clock
import os, sys
from ament_index_python.packages import get_package_share_directory
import importlib.util
from typing import List


package_dir = get_package_share_directory('cyclosafe_viewer')
launch_dir = os.path.join(package_dir, 'launch')
sys.path.insert(0, launch_dir)
from cyclosafe_config import Sensor, SensorTypeEnum

def str2bool(v):
    return v.lower() in ("yes", "true", "t", "1")

def import_sensors_list(config_path=None):
    if config_path and os.path.exists(config_path):
        module_name = os.path.basename(config_path).replace('.py', '')
        spec = importlib.util.spec_from_file_location(module_name, config_path)
        config_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(config_module)
        return config_module.sensors_list
    else:
        # Import from default config.py
        from config import sensors_list
        return sensors_list


launch_args = [
    DeclareLaunchArgument('bag', default_value=TextSubstitution(text=""), description="Specify a bag from which the data will be played"),
    DeclareLaunchArgument('map', default_value=TextSubstitution(text="false"), description="Enable live mapping from open street satellite image"),
    DeclareLaunchArgument('config', default_value=TextSubstitution(text=""), description="Optional path to a custom config file containing sensors_list"),
]


def setup_directory(parent_dir: str, time_start: float) -> str:
    pass

def launch_setup(context):
    bag = os.path.expanduser(LaunchConfiguration('bag').perform(context))
    map_arg = str2bool(LaunchConfiguration('map').perform(context))
    config_path = LaunchConfiguration('config').perform(context)

    sensors_list: List[Sensor] = import_sensors_list(config_path if config_path else None)

    rviz_config_path = os.path.join(
            get_package_share_directory('cyclosafe_viewer'),
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
            package='cyclosafe_viewer',
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
            arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0", "--roll", "-1.57", "--pitch", "1.57", "--yaw", "0", "--frame-id", "world_on_x", "--child-frame-id", "world"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["--x", "0.0", "--y", "-0.08", "--z", "0.0", "--roll", "0.00", "--pitch", "0.00", "--yaw", "0", "--frame-id", "world", "--child-frame-id", "bicycle_link"],
        ),
    ])
    gps_sensor = next((sensor for sensor in sensors_list if sensor.type == SensorTypeEnum.GPSSensor), None)
    if map_arg == True and gps_sensor != None:
        ld.extend([
            Node(
                package="cyclosafe_viewer",
                executable="gps_converter",
                parameters=[{
                    'topic': '/fix',
                    'topic_src': gps_sensor.topic
                }],
                output="screen"
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen" ,
                arguments=["--frame-id", "world", "--child-frame-id", "map"],
            ),
        ])

     # Nœuds pour le model publisher
    # Lecture du contenu du fichier URDF
    urdf_path = os.path.join(get_package_share_directory('cyclosafe_viewer'), 'urdf', 'model.urdf.xml')
    with open(urdf_path, 'r') as file:
        robot_description = file.read()
    ld.extend([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'robot_description': robot_description
            }]
        )
    ])
    return ld

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    qt_env = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')
    ld.add_action(qt_env)
    ld.add_action(opfunc)
    return ld
