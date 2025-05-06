from typing import List, Any
from std_msgs.msg import ColorRGBA
import os
from enum import IntEnum

class SensorTypeEnum(IntEnum):
	RangeSensor = 0,
	Lidar360Sensor = 1,
	GPSSensor = 2

class SerialSensor:

	colors = [
		ColorRGBA(r=1.0,g=0.0, b=0.0, a=0.25),
		ColorRGBA(r=0.0,g=1.0, b=0.0, a=0.25),
		ColorRGBA(r=0.0,g=0.0, b=1.0, a=0.25),
		ColorRGBA(r=1.0,g=1.0, b=0.0, a=0.25),
		ColorRGBA(r=0.0,g=1.0, b=1.0, a=0.25),
		ColorRGBA(r=1.0,g=1.0, b=1.0, a=0.25)
	]
	colors_str = {
		"red": colors[0],
		"green": colors[1],
		"blue": colors[2],
		"yellow": colors[3],
		"magenta": colors[4],
		"white": colors[5]
	}
	color_index = 0

	def __init__(self, package: str, executable: str, namespace: str, port: str, type: SensorTypeEnum,
			  enable: bool = True,parameters: list[Any] = [],
			  description: str = "", transform: List[str] = None, topic: str = "range", color: ColorRGBA | str = None,
			  port_hint: str = None,
			  delay: float | None = None):
		self.type = type
		self.description = description
		self.enable = enable
		self.namespace = namespace
		self.package = package
		self.executable = executable
		self.topic = f"{namespace}/{topic}"
		self.parameters = parameters
		self.transform = transform
		self.delay = delay
	
		if (port == None):
			if port_hint:
				self.port = SerialSensor.resolve_port(port_hint)
		else:
			self.port = port
		self.check_port()
	
		if color == None:
			self.color = color
		elif isinstance(color, str):
			if color == "auto":
				self.color = SerialSensor.colors[SerialSensor.color_index]
				SerialSensor.color_index = (SerialSensor.color_index + 1) % len(SerialSensor.colors)
			elif color in SerialSensor.colors_str:
				self.color = SerialSensor.colors_str[color]
			else:
				raise Exception(f"Invalid string color {color}. Valids:  {list(SerialSensor.colors_str.keys())}")
		elif not isinstance(self.color, ColorRGBA):
			raise Exception(f"Invalid color value {color}. Accept ColorRGBA object or string 'auto/red/green/...'")


	def resolve_port(hint: str) -> str | None:
		try:
			devices: List[str] = os.listdir("/dev/serial/by-id")
			matches = [dev for dev in devices if dev.find(hint) != -1]
			if len(matches) != 1:
				return None
			return (os.path.join('/dev/serial/by-id', os.readlink(os.path.join('/dev/serial/by-id', matches[0]))))
		except Exception as e:
			return None
		
	def check_port(self):
		if self.port == "" or (self.port != None and os.path.exists(self.port)):
			if self.parameters and isinstance(self.parameters[0], dict):
				if 'port' in self.parameters[0]:
					self.parameters[0]['port'] = self.port
				elif 'serial_port' in self.parameters[0]:
					self.parameters[0]['serial_port'] = self.port
			return
		self.port = None

	def get_color_int32(self) -> int:
		color = (
			((int(self.color.a * 255.0)) << 24) |
			((int(self.color.r * 255.0)) << 16) |
			((int(self.color.g * 255.0)) << 8) |
			((int(self.color.b * 255.0)) << 0)
		)
		return color

sensors_list = [
	SerialSensor(
		enable=True,
		type=SensorTypeEnum.RangeSensor,
		package="cyclosafe",
		executable="tof_lidar",
		namespace="lidar1_tof",
		description="tof lidar",
		port="/dev/ttyAMA5",
		transform=["--x", "0.18", "--y", "0.0", "--z", "0.006", "--roll", "0", "--pitch", "-1.57", "--yaw", "0", "--frame-id", "board", "--child-frame-id", "lidar1_tof/range"],
		topic="range",
		color="red",
		parameters=[{
			'baud': 921600,
			'port': "/dev/ttyAMA5",
			'period': 0.1,
		}],
	),
	SerialSensor(
		enable=True,
		type=SensorTypeEnum.RangeSensor,
		package="cyclosafe_lidar",
		executable="node_lidar",
		namespace="lidar2_tf_mini_plus",
		description="benewake tf-mini-plus lidar",
		port="/dev/ttyS0",
		transform=["--x", "0.155", "--y", "0.02", "--z", "-0.035", "--roll", "0", "--pitch", "-1.57", "--yaw", "0", "--frame-id", "board", "--child-frame-id", "lidar2_tf_mini_plus/range"],
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
	),
	SerialSensor(
		enable=True,
		type=SensorTypeEnum.RangeSensor,
		package="cyclosafe_lidar",
		executable="node_lidar",
		namespace="lidar2_tf_02",
		description="benewake tf-02 lidar",
		port='/dev/ttyS0',
		transform=["--x", "0.10", "--y", "0.026", "--z", "-0.035", "--roll", "0", "--pitch", "-1.57", "--yaw", "0", "--frame-id", "board", "--child-frame-id", "lidar2_tf_02/range"],
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
	SerialSensor(
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
	# SerialSensor(
	# 	enable=False,
	# 	type=SensorTypeEnum.RangeSensor,
	# 	package="cyclosafe",
	# 	executable="sonar_lv_pw",
	# 	namespace="sonar1",
	# 	description="sonar LV1040",
	# 	port="",
	# 	transform=["--x", "0.215", "--y", "0.0", "--z", "-0.03", "--roll", "0", "--pitch", "0", "--yaw", "0", "--frame-id","board", "--child-frame-id", "sonar1/range"],
	# 	topic="range",
	# 	color="green",
	# 	parameters=[{
	# 		'period': 0.1,
	# 	}]
	# ),
	SerialSensor(
		enable=False,
		type=SensorTypeEnum.Lidar360Sensor,
		package="ldlidar_stl_ros2",
		executable="ldlidar_stl_ros2_node",
		namespace="lidar360_1",
		description="360° lidar",
		port="/dev/ttyAMA2",
		transform=["--x", "0.045", "--y", "-0.045", "--z", "0.02", "--roll", "0", "--pitch", "0", "--yaw", "0", "--frame-id","board", "--child-frame-id", "laser1"],
		topic="scan",
		color=None,
		parameters=[{
				'product_name': 'LDLiDAR_STL27L',
				'topic_name': 'scan',
				'frame_id': 'laser1',
				'port_name': '/dev/ttyAMA2',
				'port_baudrate': 921600,
				'laser_scan_dir': False,
				'enable_angle_crop_func': False,
				'angle_crop_min': 0.0,
				'angle_crop_max': 0.0
		}],
	),
	SerialSensor(
		type=SensorTypeEnum.Lidar360Sensor,
		package="rplidar_ros",
		executable="rplidar_node",
		namespace="lidar360_2",
		description="360° lidar",
		port="/dev/ttyUSB0",
		transform=["--x", "-0.035", "--y", "-0.080", "--z", "-0.06", "--roll", "0", "--pitch", "-1.57", "--yaw", "0", "--frame-id","board", "--child-frame-id", "laser2"],
		topic="scan",
		color=None,
		parameters=[{'channel_type': 'serial',
						'serial_port': "/dev/ttyUSB0",
						'serial_baudrate': 460800,
						'frame_id': 'laser2',
						'inverted': False,
						'angle_compensate': False,
						'scan_mode': 'Standard',
						}],
	),
	# SerialSensor(
	# 	enable=True,
	# 	type=SensorTypeEnum.Lidar360Sensor,
	# 	package="rplidar_ros",
	# 	executable="rplidar_node",
	# 	namespace="lidar360_2",
	# 	description="360° lidar",
	# 	port=None,
	# 	port_hint='usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_e40dc7ec375aee118e528bdc8ffcc75d',
	# 	transform=["--x", "0.050", "--y", "-0.047", "--z", "0.02", "--roll", "0", "--pitch", "0", "--yaw", "3.14", "--frame-id","board", "--child-frame-id", "laser2"],
	# 	topic="scan",
	# 	color=None,
	# 	parameters=[{'channel_type': 'serial',
	# 					'serial_port': None,
	# 					'serial_baudrate': 460800,
	# 					'frame_id': 'laser2',
	# 					'inverted': False,
	# 					'angle_compensate': False,
	# 					'scan_mode': 'Standard',
	# 					}],
	# ),
]