import rclpy
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Range
from cyclosafe.src.ASerialSensor import ASerialPublisher

FOV = 0.3

class SonarNode(ASerialPublisher):
	
	def __init__(self):
			super().__init__('sonar', Range, 'range', '/dev/ttyUSB0', 57600)
		
	def parse(self) -> int:
		i = self.buffer.rfind(b'R')
		if (i > -1 and (len(self.buffer) - i) >= 4):
			value = int(self.buffer[i+1:i + 5].decode(('utf-8')))
			return value
		return None

	def publish(self, data: int):
		msg = Range()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = 'sonar'
		msg.field_of_view = FOV
		msg.range = float(data / 1000)
		msg.max_range = msg.range
		msg.min_range = msg.range
		msg.radiation_type = Range.ULTRASOUND
		self.pub.publish(msg)
		self.get_logger().debug(f"Published : {data}")
		

def main(args=None):
	try:
		rclpy.init(args=args)
		sensor_publisher = SonarNode()
		rclpy.spin(sensor_publisher)
	
	except (KeyboardInterrupt, ExternalShutdownException):
		pass


if __name__ == '__main__':
	main()
