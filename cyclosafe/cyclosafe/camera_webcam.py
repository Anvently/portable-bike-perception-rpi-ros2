import rclpy
from rclpy.executors import ExternalShutdownException
from cyclosafe.src.ACamera import AImagePublisher
import cv2, datetime

class ImagePublisherDesktop(AImagePublisher):
	def init_camera(self):

		self.get_logger().info(f'Starting webcam')
		self.cam = cv2.VideoCapture(0, cv2.CAP_V4L2)
		if not self.cam.isOpened():
			self.get_logger().error(f'Failed to open camera')
			raise RuntimeError(f'Failed to open camera')
		self.get_logger().info(f'Webcam started')
		self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
		self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
		self.cam.set(cv2.CAP_PROP_FPS, 30)
		self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

	def publish(self, encoding="bgr8"):
		return super().publish("bgr8")

	def capture(self):
		self.cam.grab()
		ret, frame = self.cam.retrieve()
		if not ret:
			self.get_logger().error('Failed to capture image')
		else:
			self.img_queue.append((datetime.datetime.now(), frame))
		
	def destroy(self):
		if hasattr(self, 'cam') and self.cam.isOpened():
			self.cam.release()
		super().destroy_node()

def main(args=None):
	try:
		rclpy.init(args=args)
		image_pubisher = ImagePublisherDesktop()
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


