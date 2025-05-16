
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
			  delay: float | None = None, log_level: str = "info"):
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
		self.log_level = log_level
	
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
				elif 'port_name' in self.parameters[0]:
					self.parameters[0]['port_name'] = self.port
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