import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from cyclosafe_interfaces.msg import NavSatInfo
import math

def nmea_to_decimal(nmea_coord):
	"""
	Convertit une coordonnée du format NMEA (DDMM.MMMMM) au format décimal.
	
	Args:
		nmea_coord (float): Coordonnée au format NMEA
		
	Returns:
		float: Coordonnée en format décimal
	"""

	nmea_str = str(nmea_coord)
	words = nmea_str.split('.')
	degres = 0
	if len(words[0]) <= 2:
		minutes = float(words[0] + '.' + words[1])
	else:
		degres = float(words[0][0:-2])
		minutes = float(words[0][-2:] + '.' + words[1])

	value = degres + (minutes / 60.0)

	return value

class GPSConverterNode(Node):

	def __init__(self):
		super().__init__("gps_converter")

		self.declare_parameter("topic_src", "/gps")
		self.declare_parameter("topic", "/nav_fix")

		self.topic_src = self.get_parameter("topic_src").get_parameter_value().string_value
		self.topic = self.get_parameter("topic").get_parameter_value().string_value
		
		self.publisher = self.create_publisher(NavSatFix, self.topic, 10)
		self.suscriber = self.create_subscription(NavSatInfo, self.topic_src, self.gps_callback, 10)

		self.get_logger().info("GPS message converter initialized")

	def gps_callback(self, msg: NavSatInfo):
		if msg.status.status == NavSatStatus.STATUS_FIX:
			fix = NavSatFix()
			fix.header.stamp = msg.header.stamp
			fix.status = msg.status
			fix.latitude = nmea_to_decimal(msg.latitude) if not math.isnan(fix.latitude) else fix.latitude
			fix.longitude = nmea_to_decimal(msg.longitude) if not math.isnan(fix.longitude) else fix.longitude
			self.publisher.publish(fix)

	

def main(args=None):
	try:
		rclpy.init()
		visualizer = GPSConverterNode()
		rclpy.spin(visualizer)
	except ExternalShutdownException:
		pass

if __name__ == '__main__':
	main()
	