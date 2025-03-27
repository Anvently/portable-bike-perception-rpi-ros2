import rclpy
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Range
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

class RangeCircleVisualizer(Node):
	def __init__(self):
		# Initialisation du nœud ROS
		super().__init__("cyclosafe_visualizer")
		
		# Création du publisher pour les markers RViz
		self.marker_pub = self.create_publisher(Marker, '/visualization_marker',10)
		
		# Abonnement au topic du capteur Range
		self.range_suscriber = self.create_subscription(Range, '/range', self.range_callback, 10)
		
	def range_callback(self, msg: Range):
		# print(msg, msg.range, type(msg.range))
		# Création d'un marker de type CYLINDER qui représentera notre cercle
		marker = Marker()
		marker.header.frame_id = msg.header.frame_id
		marker.header.stamp = self.get_clock().now().to_msg()
		
		# Type de marker (CYLINDER sera utilisé pour créer un cercle plat)
		marker.id = 0
		marker.type = Marker.CYLINDER
		marker.action = Marker.ADD
		
		# Position du cercle (à la position du capteur)
		marker.pose.position.x = 0.0
		marker.pose.position.y = 0.0
		marker.pose.position.z = 0.0
		# marker.scale.
		
		# Taille du cercle basée sur la distance du message Range
		marker.scale.x = msg.range * 2  # Diamètre du cercle
		marker.scale.y = msg.range * 2  # Diamètre du cercle
		marker.scale.z = 0.01  # Très fine épaisseur pour un effet 2D
		
		# Couleur du cercle (bleu semi-transparent)
		marker.color.r = 0.0
		marker.color.g = 0.0
		marker.color.b = 1.0
		marker.color.a = 0.5  # Semi-transparent
		
		# Publier le marker
		self.marker_pub.publish(marker)
		

def main(args=None):
	try:
		rclpy.init()
		visualizer = RangeCircleVisualizer()
		rclpy.spin(visualizer)
	except ExternalShutdownException:
		pass

if __name__ == '__main__':
	main()

