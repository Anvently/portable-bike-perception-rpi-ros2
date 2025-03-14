import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from serial import Serial, serialutil
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import NavSatFix
from cyclosafe.src.ASerialSensor import ASerialPublisher

class GPSPublisher(ASerialPublisher):
	
	def __init__(self):
			super().__init__(NavSatFix, 'gps', '/dev/ttyUSB0', 115200)

	def parse(self) -> NavSatFix:
		return NavSatFix()
	
	def publish(self, data: NavSatFix):
		self.pub.publish(data)

def main(args=None):
	try:
		rclpy.init(args=args)
		sensor_publisher = GPSPublisher()
		rclpy.spin(sensor_publisher)
	
	except (KeyboardInterrupt, ExternalShutdownException):
		pass


if __name__ == '__main__':
	main()