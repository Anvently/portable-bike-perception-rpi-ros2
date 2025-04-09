import rclpy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Range
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from functools import partial
from std_msgs.msg import ColorRGBA
import math
from geometry_msgs.msg import Point

class RangeCircleVisualizer(Node):

	colors = [
		ColorRGBA(r=1.0,g=0.0, b=0.0, a=0.25),
		ColorRGBA(r=0.0,g=1.0, b=0.0, a=0.25),
		ColorRGBA(r=0.0,g=0.0, b=1.0, a=0.25),
		ColorRGBA(r=1.0,g=1.0, b=0.0, a=0.25),
		ColorRGBA(r=0.0,g=1.0, b=1.0, a=0.25)
	]

	def __init__(self):
		# Initialisation du nœud ROS
		super().__init__("cyclosafe_visualizer")

		self.declare_parameter("topic_list", ["range1", "range2", "range3", "range4"])

		self.topic_list = self.get_parameter("topic_list").get_parameter_value().string_array_value
		self.suscribers = {}
		self.publisher = self.create_publisher(Marker, '/visualization_marker',10)
		
		# Création du publisher pour les markers RViz
		for topic in self.topic_list:
			self.suscribers[topic] = self.create_subscription(Range, topic, partial(self.range_callback, topic=topic), 10)
		
	def range_callback(self, msg: Range, topic: str):
		# print(msg, msg.range, type(msg.range))
		# Création d'un marker de type CYLINDER qui représentera notre cercle
		marker = Marker()
		marker.header.frame_id = topic
		marker.header.stamp = msg.header.stamp
		index = list(self.suscribers).index(topic)
		
		marker.id = index
		marker.type = Marker.POINTS
		marker.action = Marker.ADD

		# Taille des points
		marker.scale.x = 0.02
		marker.scale.y = 0.02
		
		marker.color = RangeCircleVisualizer.colors[index]
		num_points = 1000  # Nombre de points dans le cercle
		radius = msg.range
		
		for i in range(num_points):
			angle = 2.0 * 3.14159 * i / num_points
			
			point = Point()
			point.x = radius * math.cos(angle)
			point.y = radius * math.sin(angle)
			point.z = 0.0
			
			marker.points.append(point)
			
		# Publier le marker
		self.publisher.publish(marker)
		

def main(args=None):
	try:
		rclpy.init()
		visualizer = RangeCircleVisualizer()
		rclpy.spin(visualizer)
	except ExternalShutdownException:
		pass

if __name__ == '__main__':
	main()

