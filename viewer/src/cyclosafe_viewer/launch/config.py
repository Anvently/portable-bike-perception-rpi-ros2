# Created on Tue Aug 05 2025
# Updated on Tue Aug 05 2025
#
#  This file is part of Cyclosafe
# Copyright (c) 2025 Nicolas Pirard @Anvently
#
# This software is governed by the CeCILL license under French law and
# abiding by the rules of distribution of free software. You can use,
# modify and/or redistribute the software under the terms of the CeCILL
# license as circulated by CEA, CNRS and INRIA at:
# https://cecill.info/licences/Licence_CeCILL_V2.1-en.html

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
		enable=True,
		type=SensorTypeEnum.RangeSensor,
		package="cyclosafe",
		executable="tof_lidar",
		namespace="lidar1_tof",
		description="tof lidar",
		port="/dev/ttyAMA5",
		topic="range",
		color="red",
		parameters=[{
			'baud': 921600,
			'port': "/dev/ttyAMA5",
			'period': 0.05,
		}],
	),
	Sensor(
		enable=False,
		type=SensorTypeEnum.RangeSensor,
		package="cyclosafe_lidar",
		executable="node_lidar",
		namespace="lidar2_tf_mini_plus",
		description="benewake tf-mini-plus lidar",
		port="/dev/ttyS0",
		topic="range",
		color="green",
		parameters=[{
			'model': 'tf-mini-plus',
			'port': "/dev/ttyS0",
			'baud': 115200,
			'target_baud': 115200,
			'period': 0.05,
			'framerate': 20,
			'trigger': False
		}],
		log_level="info",
	),
	Sensor(
		enable=True,
		type=SensorTypeEnum.RangeSensor,
		package="cyclosafe_lidar",
		executable="node_lidar",
		namespace="lidar2_tf_02",
		description="benewake tf-02 lidar",
		port='/dev/ttyS0',
		topic="range",
		color="green",
		parameters=[{
			'model': 'tf-02',
			'port': '/dev/ttyS0',
			'baud': 115200,
			'target_baud': 115200,
			'period': 0.05,
			'framerate': 20,
			'trigger': False
		}],
		delay=5.0
	),
	Sensor(
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
		enable=True,
		type=SensorTypeEnum.Lidar360Sensor,
		package="ldlidar_stl_ros2",
		executable="ldlidar_stl_ros2_node",
		namespace="lidar360_1",
		description="360° lidar",
		port=None,
		port_hint="24cc9677b06eef11ad80b77dc169b110-if00-port0",
		topic="scan",
		color=None,
		parameters=[{
				'product_name': 'LDLiDAR_STL27L',
				'topic_name': 'scan',
				'frame_id': 'laser1',
				'port_name': None,
				'port_baudrate': 921600,
				'laser_scan_dir': False,
				'enable_angle_crop_func': False,
				'angle_crop_min': 0.0,
				'angle_crop_max': 0.0
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