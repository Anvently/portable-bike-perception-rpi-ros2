import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from serial import Serial, serialutil
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import NavSatFix
from typing import Any
from abc import abstractmethod

class ASerialPublisher(Node):
	""" 
		Abstract class for a node publishing data received via serial.
		Implements a queue system for reading incoming bytes.
		Child class must reimplement: parse(), publish()
	"""
	
	def __init__(self, message_type: Any, topic: str, default_port: str, default_baud: int):
			super().__init__('sensor_publisher')	
			self.port = default_port
			self.baud = default_baud
			self.period = None
			self.buffer = bytes()
			self.serial = None

			self.declare_parameter('port', default_port, ParameterDescriptor(description="device from which the serial data will be read"))
			self.declare_parameter('baud', default_baud, ParameterDescriptor(description="serial interface baudrate"))
			self.declare_parameter('period', 0.5, ParameterDescriptor(description="serial interface baudrate"))

			self.pub = self.create_publisher(message_type, topic, 10)

			self.timer = self.create_timer(0, self.timer_callback)
			self.count = 0

	def update_parameters(self):
		port = self.get_parameter('port').get_parameter_value().string_value
		baud = self.get_parameter('baud').get_parameter_value().integer_value
		if (port != self.port or baud != self.baud):
			self.port = port
			self.baud = baud
			self.serial: Serial = Serial(self.port, self.baud)
			self.buffer = bytes()
			self.get_logger().info(f"listening on port {self.port}, baud={self.baud}")
		period = self.get_parameter('period').get_parameter_value().double_value
		if self.period != period:
			self.period = period
			if (hasattr(self, 'timer')):
				self.timer.timer_period_ns = self.period * 1000 * 1000 * 1000

	def timer_callback(self):
		try:
			self.update_parameters()
			if not self.serial:
				self.serial: Serial = Serial(self.port, self.baud)
				self.get_logger().info(f"listening on port {self.port}, baud={self.baud}")
			data = self.read()
			if data:
				self.buffer = bytes()
				self.publish(data)

		except (serialutil.SerialException, OSError) as e:
			self.get_logger().error(f"Failed to read from serial: {e.strerror}\nRetrying...")
			self.timer.timer_period_ns = 2 * 1000 * 1000 * 1000
			self.serial = None
	
	@abstractmethod
	def publish(self, data: Any):
		raise Exception("Abstract method not implemented")
	
	@abstractmethod
	def parse(self) -> Any:
		"""Must use bytes buffer to retrieve informations and extract data.
		Data will be given at publish() method and buffer will be emptied.
		Return none if no data could be extracted from buffer."""
		raise Exception("Abstract method not implemented")

	def read(self) -> Any:
		'''Return the last value sent from sensor'''
		if (self.serial.in_waiting):
			self.buffer += self.serial.read(self.serial.in_waiting)
			value = self.parse()
			return value
		return None
			