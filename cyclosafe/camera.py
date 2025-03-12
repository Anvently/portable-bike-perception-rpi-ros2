import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import CompressedImage
import io, time
from picamera2 import Picamera2

class ImagePublisher(Node):

	def __init__(self):
		super().__init__('camera_publisher')
		self.cam = Picamera2()
		self.capture_config = self.cam.create_still_configuration()
		self.configure(self.cam.capture_config)
		self.format = "jpeg"
		self.quality = 80
		self.cam.start()

		time.sleep(2)

		self.pub = self.create_publisher(CompressedImage, 'images', 10)
		self.timer = self.create_timer(1, self.capture)
		self.job = None
		self.get_logger().info('Camera publisher node started')


	def capture(self):
		try:
			msg = CompressedImage()
			msg.header.stamp = self.get_clock().now().to_msg()
			msg.format = self.format
	
			data = io.BytesIO()
			self.job = self.cam.capture_file(data, self.capture_config, format=self.format, quality=self.quality, wait=False)
			self.cam.wait(self.job)
			data.seek(0)
			msg.data = data.getvalue()

			self.pub.publish(msg)
			self.get_logger().debug(f"Published image: {len(msg.data)} bytes")
			
		except Exception as e:
			self.get_logger().error(f"Failed to capture image: {str(e)}")
			

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

