from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration
# from rclpy.time import Time
# from rclpy.clock import Clock
import datetime, time
import os

def str2bool(v):
    return v.lower() in ("yes", "true", "t", "1")

launch_args = [
    DeclareLaunchArgument('out_path', default_value=TextSubstitution(text=""), description="Path in which a data directory for this simulation will be created"),
    DeclareLaunchArgument('log_level', default_value=TextSubstitution(text="info"), description="Log level for all nodes"),
    DeclareLaunchArgument('record', default_value=TextSubstitution(text="false"), description="Capture every topic in a bag"),
    DeclareLaunchArgument('save', default_value=TextSubstitution(text="true"), description="If true, hub node won't be started and data will not be written. Use in with record=true in order to only record data from ROS perspective."),
]

def setup_directory(parent_dir: str, time_start: float) -> str:
    if not parent_dir:
        parent_dir = os.path.join(os.getenv("HOME", ""), "data")
    
    os.makedirs(parent_dir, exist_ok=True)
    
    path = os.path.join(parent_dir, time.strftime("%Y%m%d-%H%M%S", time.gmtime(time_start)))

    try:
        os.mkdir(path, 0o711) 
    except FileExistsError:
        raise Exception(f"Directory {path} already exists.")
    latest_link = os.path.join(parent_dir, "latest")
    try:
        if os.path.islink(latest_link):
            os.unlink(latest_link)
        
        os.symlink(path, latest_link)
    except OSError as e:
        print(f"Warning: Could not create symlink: {e}")
    os.mkdir(f"{path}/logs", 511)
    return (path)

def launch_setup(context):
    parent_dir = LaunchConfiguration('out_path').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    record = str2bool(LaunchConfiguration('record').perform(context))
    save = str2bool(LaunchConfiguration('save').perform(context))
    time_start = time.time()

    path = setup_directory(parent_dir, time_start)
    print(f"Simulation start time = {time_start}")
    ld = []
    if record:
        ld.extend([
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-a', '-o', os.path.join(path, "bag")],
                output='screen'
            )
        ])
    if save:
        ld.extend([Node(
            package='cyclosafe_hub',
            executable='hub',
            namespace='',
            output='screen',
            emulate_tty=True,
            parameters=[
                { 'start_time': float(time_start),
                 'out_path': path}
            ],
            arguments=['--ros-args', '--log-level', log_level],
        )])
    ld.extend([SetEnvironmentVariable(name='ROS_LOG_DIR', value=os.path.join(path, "logs")),
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
            ],
            arguments=['--ros-args', '--log-level', log_level],
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
            ],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        Node(
            package='cyclosafe',
            executable='sonar',
            namespace='',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'baud': 57600,
                 'port': '/dev/ttyUSB1',
                 'period': 0.20,
                 'start_time': float(time_start)}
            ],
            arguments=['--ros-args', '--log-level', log_level],
		),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type': 'serial',
                         'serial_port': '/dev/ttyUSB0',
                         'serial_baudrate': 460800,
                         'frame_id': 'laser',
                         'inverted': False,
                         'angle_compensate': False,
                         'scan_mode': 'Standard',
                         }],
            output='screen'
        ),
    ])
    return ld

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
