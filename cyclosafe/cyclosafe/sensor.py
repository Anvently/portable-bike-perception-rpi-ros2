import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String as StringMsg
from serial import Serial, serialutil
from collections import deque
from cyclosafe.src.ASerialSensor import ASerialPublisher

class SensorPublisher(ASerialPublisher):
	
	def __init__(self):
			super().__init__(StringMsg, 'range', '/dev/ttyUSB0', 115200)
		
	def parse(self) -> str:
		i = self.buffer.rfind(b'R')
		if (i > -1 and (len(self.buffer) - i) >= 4):
			value = self.buffer[i:i + 4].decode(('utf-8'))
			return value
		return None

	def publish(self, data: str):
		msg = StringMsg()
		msg.data = data
		self.pub.publish(msg)
		self.get_logger().info(f"Published : {data}")
		

def main(args=None):
	try:
		rclpy.init(args=args)
		sensor_publisher = SensorPublisher()
		rclpy.spin(sensor_publisher)
	
	except (KeyboardInterrupt, ExternalShutdownException):
		pass


if __name__ == '__main__':
	main()
