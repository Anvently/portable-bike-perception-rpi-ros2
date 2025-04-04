import rclpy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Range
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from functools import partial
from std_msgs.msg import ColorRGBA

class SonarRepublisher(Node):

	colors = [
		ColorRGBA(r=1.0,g=0.0, b=0.0, a=0.25),
		ColorRGBA(r=0.0,g=1.0, b=0.0, a=0.25),
		ColorRGBA(r=0.0,g=0.0, b=1.0, a=0.25),
		ColorRGBA(r=1.0,g=1.0, b=0.0, a=0.25),
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
		cylinder = Marker()
		cylinder.header.frame_id = topic
		cylinder.header.stamp = self.get_clock().now().to_msg()
		
		# Type de cylinder (CYLINDER sera utilisé pour créer un cercle plat)
		cylinder.id = 0
		cylinder.type = Marker.CYLINDER
	
		cylinder.action = Marker.ADD
		
		# Position du cercle (à la position du capteur)
		cylinder.pose.position.x = 0.0
		cylinder.pose.position.y = 0.0
		cylinder.pose.position.z = 0.0
		# cylinder.scale.
		
		# Taille du cercle basée sur la distance du message Range
		cylinder.scale.x = msg.range * 2  # Diamètre du cercle
		cylinder.scale.y = msg.range * 2  # Diamètre du cercle
		cylinder.scale.z = 0.01  # Très fine épaisseur pour un effet 2D
		
		index = list(self.suscribers).index(topic)

		cylinder.color = SonarRepublisher.colors[index]

		# Publier le cylinder
		cone = Marker()
		cone.header.frame_id = topic
		cone.header.stamp = self.get_clock().now().to_msg()
		cone.id = 1  # ID unique pour le cône
		cone.type = Marker.ARROW  # ARROW peut être utilisé pour créer un cône
		cone.action = Marker.ADD

		# Position du cône (juste au-dessus du cylindre)
		cone.pose.position.x = 0.0
		cone.pose.position.y = 0.0
		cone.pose.position.z = 0.1  # Positionné sur le dessus du cylindre

		# Orientation du cône (pointant vers le haut)
		cone.pose.orientation.x = 0.0
		cone.pose.orientation.y = 0.0
		cone.pose.orientation.z = 0.0
		cone.pose.orientation.w = 1.0

		# Taille du cône
		cone.scale.x = 0.2  # Longueur
		cone.scale.y = msg.range  # Largeur à la base
		cone.scale.z = msg.range  # Hauteur

		# Couleur du cône (même que le cylindre)
		cone.color = SonarRepublisher.colors[index]

		self.publisher.publish(cylinder)
		

def main(args=None):
	try:
		rclpy.init()
		visualizer = SonarRepublisher()
		rclpy.spin(visualizer)
	except ExternalShutdownException:
		pass

if __name__ == '__main__':
	main()

