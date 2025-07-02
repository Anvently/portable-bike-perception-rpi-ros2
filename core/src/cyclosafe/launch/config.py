from typing import List, Any
from std_msgs.msg import ColorRGBA
import os
from enum import IntEnum
from cyclosafe_config import Sensor, SensorTypeEnum

sensors_list = [
	Sensor(
		enable=True,
		type=SensorTypeEnum.CameraSensor,
		package="cyclosafe",
		executable="camera_pi",
		namespace="",
		description="camera node",
		port="",
		topic="images/compressed",
		color=None,
		parameters=[{'queue_size': 0,
            'resolution': [600, 400],
            'interval': 0.1,
            'compression': 90,
            'preview': False,
        }],
	),
	Sensor(
		enable=False,
		type=SensorTypeEnum.GPSSensor,
		package="cyclosafe",
		executable="gps",
		namespace="",
		description="gps node",
		port="/dev/ttyACM0",
		transform=None,
		topic="gps",
		color=None,
		parameters=[{
				'baud': 115200,
				'port': "/dev/ttyACM0",
				'period': 1.0,
		}],
	),
	Sensor(
		type=SensorTypeEnum.Lidar360Sensor,
		package="rplidar_ros",
		executable="rplidar_node",
		namespace="lidar360_2",
		description="360° lidar",
		port=None,
		port_hint="e40dc7ec375aee118e528bdc8ffcc75d-if00-port0",
		topic="scan",
		color=None,
		parameters=[{'channel_type': 'serial',
						'serial_port': None,
						'serial_baudrate': 460800,
						'frame_id': 'laser2',
						'inverted': False,
						'angle_compensate': False,
						'scan_mode': 'Standard',
						}],
	),
]