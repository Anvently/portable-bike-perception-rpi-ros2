import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CompressedImageViewer(Node):
	def __init__(self):
		super().__init__('compressed_image_viewer')
		
		# Créer un subscriber pour le topic CompressedImage
		self.subscription = self.create_subscription(
			CompressedImage, 
			'images/compressed', 
			self.image_callback, 
			10  # Profondeur de la file d'attente
		)
		
		# Empêcher l'optimisation du subscriber par le compilateur
		self.subscription
		
		# Message de log pour confirmer que le nœud est démarré
		self.get_logger().info('Compressed Image Viewer Node started')
		
	def image_callback(self, msg):
		try:
			# Convertir le message CompressedImage en tableau NumPy
			np_arr = np.frombuffer(msg.data, np.uint8)
			
			# Décoder l'image compressée
			frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
			
			if frame is not None:
				# Afficher l'image dans une fenêtre
				cv2.imshow('Compressed Image Viewer', frame)
				
				# Attendre 1 ms et vérifier si l'utilisateur veut quitter (touche 'q')
				key = cv2.waitKey(1) & 0xFF
				if key == ord('q'):
					self.get_logger().info('Closing viewer...')
					rclpy.shutdown()
			
		except Exception as e:
			self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
	# Initialiser ROS2
	rclpy.init(args=args)
	
	try:
		# Créer et exécuter le nœud
		viewer = CompressedImageViewer()
		rclpy.spin(viewer)
	
	except KeyboardInterrupt:
		pass
	
	finally:
		# Nettoyage
		viewer.destroy_node()
		cv2.destroyAllWindows()
		rclpy.shutdown()

if __name__ == '__main__':
	main()