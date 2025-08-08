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
# https://cecill.info/licences/Licence_CeCILL-B_V1-en.html


from rclpy.node import Node
from serial import Serial, serialutil
from rcl_interfaces.msg import ParameterDescriptor
from typing import Any
from abc import abstractmethod
from rclpy.time import Time
import logging, time

MS_TO_NS = (1000 * 1000)

class ASerialPublisher(Node):
	""" 
		Abstract class for a node publishing data received via serial.
		Implements a queue system for reading incoming bytes.
		Child class must reimplement: parse(), publish()
	"""
	
	def __init__(self, name: str, message_type: Any, topic: str, default_port: str, default_baud: int):
			super().__init__(name)
			self.port = default_port
			self.baud = default_baud
			self.period = None
			self.buffer = bytes()
			self.serial = None

			self.declare_parameter('port', default_port, ParameterDescriptor(description="device from which the serial data will be read"))
			self.declare_parameter('baud', default_baud, ParameterDescriptor(description="serial interface baudrate"))
			self.declare_parameter('period', 0.5, ParameterDescriptor(description="serial interface baudrate"))
			self.declare_parameter('start_time', 0.0, ParameterDescriptor(description="Time to be used as the beginning of the simulation. Float value of seconds since epoch."))
			self.declare_parameter('unit', "mm", ParameterDescriptor(description="Unit of data sent by sensor. Accept 'mm', 'm' or 'in'"))
			self.period = self.get_parameter('period').get_parameter_value().double_value
			self.port = self.get_parameter('port').get_parameter_value().string_value
			self.baud = self.get_parameter('baud').get_parameter_value().integer_value
			self.unit = self.get_parameter('unit').get_parameter_value().string_value
			match self.unit:
				case 'mm':
					self.factor = 0.001
				case 'm':
					self.factor = 1
				case 'in':
					self.factor = 0.0254
				case _:
					raise Exception(f"Invalid unit {self.unit}, accepts: 'mm', 'm' or 'in'")


			self.start_time = Time(seconds=self.get_parameter('start_time').get_parameter_value().double_value, clock_type=self.get_clock().clock_type)

			self.pub = self.create_publisher(message_type, topic, 10)

			self.timer = self.create_timer(0, self.routine)
			self.count = 0
			logging.basicConfig(format='%(asctime)s %(levelname)-8s %(message)s',
					level=logging.INFO,
					datefmt='%Y-%m-%d %H:%M:%S')


	def	get_current_timestamp(self) -> int:
		"""Return the current time in ms from the beginning of the simulation"""
		now: Time = self.get_clock().now()
		return (int((now - self.start_time).nanoseconds / MS_TO_NS))
	
	def routine(self):
		try:
			if not self.serial:
				self.serial: Serial = Serial(self.port, self.baud)
				self.get_logger().info(f"listening on port {self.port}, baud={self.baud}")
				self.timer.timer_period_ns = self.period * 1000 * 1000 * 1000
			data = self.read()
			if data:
				self.buffer = bytes()
				self.publish(data)

		except (serialutil.SerialException, OSError) as e:
			self.get_logger().error(f"Failed to read from serial: {e.strerror}\nRetrying...")
			self.timer.timer_period_ns = 10 * 1000 * 1000 * 1000
			self.serial = None

		except Exception as e:
			self.get_logger().error(f"Exception while parsing: {str(e)}")
			self.timer.timer_period_ns = 3 * 1000 * 1000 * 1000
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
			