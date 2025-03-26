from rclpy.node import Node, ParameterDescriptor
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from collections import deque
from abc import abstractmethod
from cyclosafe_interfaces.srv import SaveImages
from enum import IntEnum
import os, ctypes
import datetime, cv2
import time
from rclpy.time import Time, S_TO_NS

MS_TO_NS = (1000 * 1000)

IMAGES_PATH = os.getenv("HOME") + "/data/images/"

class SaveFilesResult(IntEnum):
	SUCCESS = 0
	PARTIAL_SUCCESS = 1
	FAILURE = 2

class AImagePublisher(Node):

	def __init__(self):
		super().__init__('camera_publisher')

		try:

			self.declare_parameter('queue_size', 20, ParameterDescriptor(description="Max number of images stored in memory."))
			self.declare_parameter('resolution', [800, 600], ParameterDescriptor(description="Image resolution: [width, height]"))
			self.declare_parameter('interval', 0.5, ParameterDescriptor(description="Interval during each image"))
			self.declare_parameter('compression', 95, ParameterDescriptor(description="Compression level [0-100]"))
			self.declare_parameter('preview', True, ParameterDescriptor(description="Enable/Disable preview. Only before start"))
			self.declare_parameter('start_time', 0.0, ParameterDescriptor(description="Time to be used as the beginning of the simulation. Float value of seconds since epoch."))
			self.update_parameters()

			self.start_time = Time(seconds=self.get_parameter('start_time').get_parameter_value().double_value, clock_type=self.get_clock().clock_type)

			self.img_queue = deque(maxlen=self.queue_size)
			self.bridge = CvBridge()
			self.count = 0

			self.init_camera()
			
			self.pub = self.create_publisher(CompressedImage, 'images/compressed', 10)
			self.save_service = self.create_service(SaveImages, 'save_images', self.save_files)
			self.timer = self.create_timer(0.2, self.routine)
		
		except RuntimeError as e:
			self.get_logger().error(f'Failed to init camera: {str(e)}')
			raise e
	
		self.get_logger().info('Camera publisher node started')

	@abstractmethod
	def init_camera(self):
		pass

	@abstractmethod
	def capture(self):
		pass

	def	get_current_timestamp(self) -> int:
		"""Return the current time in ms from the beginning of the simulation"""
		now: Time = self.get_clock().now()
		return (int((now - self.start_time).nanoseconds / MS_TO_NS))

	def save_files(self, request: SaveImages.Request, response : SaveImages.Response) -> SaveImages.Response:
		try:
			if (len(self.img_queue) == 0):
				raise Exception("no image available")
			now = self.get_current_timestamp()
			delta: int = now - self.img_queue[0][0]
			if (delta >= request.time):
				response.result = SaveFilesResult.SUCCESS
			else:
				response.result = SaveFilesResult.PARTIAL_SUCCESS

			for i in range(0, len(self.img_queue)):
				(img_timestamp, img_data) = self.img_queue[i]
				if (now - img_timestamp > request.time):
					continue
				path: str = f"{request.path}/{img_timestamp}.jpg"
				if (os.path.isfile(path)):
					continue
				if (cv2.imwrite(path, cv2.cvtColor(img_data, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, self.compression]) == False):
					raise Exception("file creation failed, is th directory created ?")
	
		except Exception as e:
			self.get_logger().error(f"SaveFiles service: Failed to save images: {str(e)}")
			response.result = SaveFilesResult.FAILURE
		return response

	def publish(self, encoding="rgb8"):
		if (len(self.img_queue) > 0):
			# Récupérer l'image la plus récente
			_, img_data = self.img_queue[-1]
			
			# Encoder l'image en JPEG avec le niveau de compression défini
			encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.compression]
			_, compressed_img = cv2.imencode('.jpg', cv2.cvtColor(img_data, cv2.COLOR_RGB2BGR), encode_param)
			# Créer un message CompressedImage
			msg = CompressedImage()
			msg.header.stamp = self.get_clock().now().to_msg()
			msg.format = 'jpeg'
			msg.data = compressed_img.tobytes()
			
			# Publier le message
			self.pub.publish(msg)
			self.get_logger().debug(f"Published compressed image: {len(msg.data) / 1024:.2f}KB at {self.img_queue[-1][0]}")

	def update_parameters(self):
		self.compression = self.get_parameter('compression').get_parameter_value().integer_value
		# self.queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
		queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
		if (hasattr(self, 'img_queue') and queue_size != self.queue_size):
			self.img_queue = deque(maxlen=self.queue_size)
		self.queue_size = queue_size
		self.interval = self.get_parameter('interval').get_parameter_value().double_value
		if (hasattr(self, 'timer')):
			self.timer.timer_period_ns = self.interval * 1000 * 1000 * 1000
		self.resolution = self.get_parameter('resolution').get_parameter_value().integer_array_value

	def routine(self):
		try:
			self.update_parameters()
			self.capture()
			self.publish()
			self.count += 1
		except Exception as e:
			self.get_logger().error(f"Failed to capture image: {str(e)}")