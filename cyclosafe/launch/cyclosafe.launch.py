from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration
import time
import os, sys

package_dir = get_package_share_directory('cyclosafe')
launch_dir = os.path.join(package_dir, 'launch')
sys.path.insert(0, launch_dir)
from config import sensors_list, SensorTypeEnum

def str2bool(v):
    return v.lower() in ("yes", "true", "t", "1")

launch_args = [
    DeclareLaunchArgument('out_path', default_value=TextSubstitution(text=""), description="Path in which a data directory for this simulation will be created"),
    DeclareLaunchArgument('log_level', default_value=TextSubstitution(text="info"), description="Log level for all nodes"),
    DeclareLaunchArgument('record', default_value=TextSubstitution(text="false"), description="Capture every topic in a bag"),
    DeclareLaunchArgument('save', default_value=TextSubstitution(text="false"), description="If true, hub node won't be started and data will not be written. Use in with record=true in order to only record data from ROS perspective."),
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

def launch_setup(context):
    parent_dir = LaunchConfiguration('out_path').perform(context)
    log_level = LaunchConfiguration('log_level').perform(context)
    record = str2bool(LaunchConfiguration('record').perform(context))
    save = str2bool(LaunchConfiguration('save').perform(context))
    time_start = time.time()
    path = setup_directory(parent_dir, time_start)
    print(f"Simulation start time = {time_start}")

    ld = []
    ld.extend([SetEnvironmentVariable(name='ROS_LOG_DIR', value=os.path.join(path, "logs"))])
    if record:
        ld.extend([
            ExecuteProcess(
                cmd=['ros2', 'bag', 'record', '-a', '-b', '50000000', '--compression-mode', 'file', '--compression-format', 'zstd', '-o', os.path.join(path, "bag")],
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
    ld.extend([Node(
        package='cyclosafe',
        executable='camera_pi',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[
        {'queue_size': 0,
            'resolution': [600, 400],
            'interval': 0.1,
            'compression': 90,
            'preview': False,
            'start_time': float(time_start)}
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )])
    for sensor in sensors_list:
        if sensor.enable == False or sensor.port == None:
            continue
        params = sensor.parameters.copy()
        params.append({'start_time': float(time_start)})

        list = [Node(
            package=sensor.package,
            executable=sensor.executable,
            namespace=sensor.namespace,
            output='screen',
            emulate_tty=True,
            parameters=params,
            arguments=['--ros-args', '--log-level', log_level],
        )]
        if sensor.delay != None:
            print(f"adding delay of {sensor.delay}")
            ld.extend([TimerAction(period=sensor.delay, actions=list)])
        else:
            ld.extend(list)
    return ld

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)

    ld.add_action(opfunc)
    return ld
