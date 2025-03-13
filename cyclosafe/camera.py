import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
import io, time
from picamera2 import Picamera2, Preview
from libcamera import Transform
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):

	def __init__(self):
		super().__init__('camera_publisher')
		self.cam = Picamera2()
		self.bridge = CvBridge()
		capture_config = self.cam.create_video_configuration(main={"size": (1200, 800), "format": "RGB888"}, transform=Transform(vflip=True))
		self.cam.configure(capture_config)
		self.cam.start_preview(Preview.DRM)
		self.count = 0
		self.quality = 100
		#self.cam.options["quality"] = self.quality
		#self.cam.options["compress_level"] = 2
		self.cam.start()

		#time.sleep(2)

		self.pub = self.create_publisher(Image, 'images', 10)
		self.timer = self.create_timer(0.2, self.capture)
		self.get_logger().info('Camera publisher node started')


	def capture(self):
		try:
			img = self.cam.capture_array()
			cv2.imwrite(f"/home/npirard/data/images/{self.count}.jpg", cv2.cvtColor(img, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, self.quality])
			self.pub.publish(self.bridge.cv2_to_imgmsg(img, "rgb8"))
			self.get_logger().info(f"Published image")
			self.count += 1
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

