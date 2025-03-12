import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from serial import Serial, serialutil
from collections import deque

class SensorPublisher(Node):
	
	def __init__(self):
			super().__init__('sensor_publisher')
			self.pub = self.create_publisher(String, 'range_data', 10)
			timer_period = 0.5
			self.timer = self.create_timer(timer_period, self.timer_callback)
			self.count = 0
			self.port = None
			self.baud = None
			self.values = deque()
			self.buffer = bytes()
			self.serial = None

			self.declare_parameter('port', '/dev/ttyUSB0', ParameterDescriptor(description="device from which the serial data will be read"))
			self.declare_parameter('baud', 57600, ParameterDescriptor(description="serial interface baudrate"))

	def timer_callback(self):
		port = self.get_parameter('port').get_parameter_value().string_value
		baud = self.get_parameter('baud').get_parameter_value().integer_value
		try:
			if (self.serial and self.port == port and baud == self.baud):
				data: str = self.read()
				if data:
					msg = String()
					msg.data = f"{self.serial.port}: {data}"
					self.pub.publish(msg)
					self.get_logger().info(f"Publishing: {msg.data}")
					self.count += 1
			else:
				self.port = port
				self.baud = baud
				self.serial: Serial = Serial(self.port, self.baud)
				self.timer.timer_period_ns = 0.5 * 1000 * 1000 * 1000
		except (serialutil.SerialException, OSError) as e:
			# print(f"Failed to read from serial: {e.strerror}\nRetrying...")
			self.get_logger().error(f"Failed to read from serial: {e.strerror}\nRetrying...")
			self.timer.timer_period_ns = 2 * 1000 * 1000 * 1000
			self.serial = None
		

	def read(self) -> str:
		'''Return the last value sent from sensor'''
		if (self.serial.in_waiting):
			self.buffer += self.serial.read(self.serial.in_waiting)
			i = self.buffer.rfind(b'R')
			if (i > -1 and (len(self.buffer) - i) >= 4):
				value = self.buffer[i:i + 4].decode(('utf-8'))
				self.buffer = bytes()
				return value
		return None
			
		

def main(args=None):
	try:
		rclpy.init(args=args)
		sensor_publisher = SensorPublisher()
		rclpy.spin(sensor_publisher)
	
	except (KeyboardInterrupt, ExternalShutdownException):
		pass


if __name__ == '__main__':
	main()
