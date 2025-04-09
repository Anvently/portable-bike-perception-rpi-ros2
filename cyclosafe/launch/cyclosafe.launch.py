from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration
# from rclpy.time import Time
# from rclpy.clock import Clock
import datetime, time
import os
from typing import List, Tuple

DFT_LIDAR_PORT = "/dev/ttyAMA2"
DFT_GPS_PORT = "/dev/ttyACM0"
DFT_SONAR1_PORT = "/dev/ttyUSB0"
DFT_SONAR2_PORT = "/dev/ttyUSB1"
DFT_SONAR3_PORT = "/dev/ttyACM0"
DFT_SONAR4_PORT = "/dev/ttyACM1"

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

    while (1):
        try:
            os.mkdir(path, 0o711)
            break
        except FileExistsError:
            path += "-2"

    latest_link = os.path.join(parent_dir, "latest")
    try:
        if os.path.islink(latest_link):
            os.unlink(latest_link)
        
        os.symlink(path, latest_link)
    except OSError as e:
        print(f"Warning: Could not create symlink: {e}")
    os.mkdir(f"{path}/logs", 511)
    return (path)

def read_port_by_id(hint: str) -> str:
    try:
        devices: List[str] = os.listdir("/dev/serial/by-id")
        matches = [dev for dev in devices if dev.find(hint) != -1]
        if len(matches) != 1:
            return None
        return (os.path.join('/dev/serial/by-id', os.readlink(os.path.join('/dev/serial/by-id', matches[0]))))
    except Exception as e:
        return None

def resolve_port() -> Tuple[str, str, str]:
    port_lidar: str = DFT_LIDAR_PORT
    port_gps: str = DFT_GPS_PORT

    port = read_port_by_id('CP2102N')
    if port: port_lidar = port
    port = read_port_by_id('u-blox')
    if port: port_gps = port
    port_sonar = read_port_by_id('MaxBotix_MB1443')

    return port_lidar, port_gps, port_sonar

def resolve_port_sonar() -> Tuple[str, str, str]:
    port_sonar2 = read_port_by_id('MaxBotix_MB1433')
    port_sonar3 = read_port_by_id('MaxBotix_MB1423')
    port_sonar4 = read_port_by_id('MaxBotix_MB1413')

    return port_sonar2, port_sonar3, port_sonar4


def launch_setup(context):
    parent_dir = LaunchConfiguration('out_path').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    record = str2bool(LaunchConfiguration('record').perform(context))
    save = str2bool(LaunchConfiguration('save').perform(context))
    time_start = time.time()

    path = setup_directory(parent_dir, time_start)
    port_lidar, port_gps, port_sonar1 = resolve_port()
    port_sonar2, port_sonar3, port_sonar4 = resolve_port_sonar()
    port_sonar5 = "/dev/ttyS0"
    print(port_lidar, port_gps, port_sonar1, port_sonar2, port_sonar3, port_sonar4)
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
    if port_sonar1:
        ld.extend([Node(
            package='cyclosafe',
            executable='sonar',
            namespace='sonar1',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'baud': 57600,
                 'port': port_sonar1,
                 'period': 0.15,
                 'start_time': float(time_start)}
            ],
            arguments=['--ros-args', '--log-level', log_level],
		)])
    if port_sonar2:
        ld.extend([Node(
            package='cyclosafe',
            executable='sonar',
            namespace='sonar2',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'baud': 57600,
                 'port': port_sonar2,
                 'period': 0.15,
                 'start_time': float(time_start)}
            ],
            arguments=['--ros-args', '--log-level', log_level],
		)])
    if port_sonar3:
        ld.extend([Node(
            package='cyclosafe',
            executable='sonar',
            namespace='sonar3',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'baud': 57600,
                 'port': port_sonar3,
                 'period': 0.15,
                 'start_time': float(time_start)}
            ],
            arguments=['--ros-args', '--log-level', log_level],
		)])
    if port_sonar4:
        ld.extend([Node(
            package='cyclosafe',
            executable='sonar',
            namespace='sonar4',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'baud': 57600,
                 'port': port_sonar4,
                 'period': 0.15,
                 'start_time': float(time_start)}
            ],
            arguments=['--ros-args', '--log-level', log_level],
		)])
    if os.path.exists(port_sonar5):
        ld.extend([Node(
            package='cyclosafe',
            executable='sonar',
            namespace='sonar5',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'baud': 9600,
                 'port': port_sonar5,
                 'period': 0.15,
                 'start_time': float(time_start)}
            ],
            arguments=['--ros-args', '--log-level', log_level],
		)])
    ld.extend([SetEnvironmentVariable(name='ROS_LOG_DIR', value=os.path.join(path, "logs")),
        # Node(
        #     package='cyclosafe',
        #     executable='gps',
        #     namespace='',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[
        #         {'baud': 115200,
        #          'port': port_gps,
        #          'start_time': float(time_start)}
        #     ],
        #     arguments=['--ros-args', '--log-level', log_level],
        # ),
        Node(
            package='cyclosafe',
            executable='camera_pi',
            namespace='',
            output='screen',
            emulate_tty=True,
            parameters=[
               {'queue_size': 10,
                 'resolution': [600, 800],
                 'interval': 0.25,
                 'compression': 95,
                 'preview': False,
                 'start_time': float(time_start)}
            ],
            arguments=['--ros-args', '--log-level', log_level],
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{'channel_type': 'serial',
                         'serial_port': port_lidar,
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
