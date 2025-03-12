import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
import io, time, cv2, cv_bridge


class ImagePublisher(Node):
	def __init__(self):
		super().__init__('camera_publisher')
		
		# Déclarer les paramètres avec des valeurs par défaut
		self.declare_parameter('camera_id', 0, 
							ParameterDescriptor(description='Camera device ID'))
		self.declare_parameter('width', 640, 
							ParameterDescriptor(description='Image width'))
		self.declare_parameter('height', 480, 
							ParameterDescriptor(description='Image height'))
		self.declare_parameter('fps', 30.0, 
							ParameterDescriptor(description='Camera FPS'))
		
		# Récupérer les valeurs des paramètres
		self.camera_id = self.get_parameter('camera_id').value
		self.width = self.get_parameter('width').value
		self.height = self.get_parameter('height').value
		self.fps = self.get_parameter('fps').value
		
		# Initialiser cv_bridge pour convertir entre OpenCV et ROS
		self.bridge = cv_bridge.CvBridge()
		
		# Initialiser la caméra OpenCV
		self.cap = cv2.VideoCapture(self.camera_id)
		if not self.cap.isOpened():
			self.get_logger().error(f'Failed to open camera with ID {self.camera_id}')
			raise RuntimeError(f'Failed to open camera with ID {self.camera_id}')
		
		# Configurer les propriétés de la caméra
		self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
		self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
		self.cap.set(cv2.CAP_PROP_FPS, self.fps)
		
		# Vérifier les valeurs réelles obtenues
		actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
		actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
		actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
		
		self.get_logger().info(f'Camera initialized with resolution: {actual_width}x{actual_height}, FPS: {actual_fps}')
		
		# Créer le publisher pour les images raw (non compressées)
		self.pub = self.create_publisher(Image, 'image_raw', 10)
		
		# Configurer le timer pour prendre des photos toutes les secondes
		self.timer = self.create_timer(0.0, self.capture)
		
		self.get_logger().info('Raw image publisher node started')

	def capture(self):
		try:
			# Capturer une image
			ret, frame = self.cap.read()
			
			if not ret:
				self.get_logger().error('Failed to capture image')
				return
			
			# Convertir l'image OpenCV en message ROS Image
			msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
			msg.header.stamp = self.get_clock().now().to_msg()
			
			# Publier le message
			self.pub.publish(msg)
			
			height, width, _ = frame.shape
			self.get_logger().info(f'Published raw image: {width}x{height} ({len(msg.data)})')
			
		except Exception as e:
			self.get_logger().error(f'Error capturing image: {str(e)}')

	def destroy_node(self):
		# Libérer les ressources de la caméra lors de la fermeture
		if hasattr(self, 'cap') and self.cap.isOpened():
			self.cap.release()
		super().destroy_node()

def main(args=None):
	try:
		rclpy.init(args=args)
		image_pubisher = ImagePublisher()
		rclpy.spin(image_pubisher)
	
	except (KeyboardInterrupt, ExternalShutdownException):
		pass

	finally:
		if 'image_publisher' in locals():
			image_pubisher.cam.stop()
			image_pubisher.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()


