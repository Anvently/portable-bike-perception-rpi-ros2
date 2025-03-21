from rclpy.node import Node, ParameterDescriptor
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from collections import deque
from abc import abstractmethod
from cyclosafe_interfaces.srv import SaveImages
from enum import IntEnum
import os, ctypes
import datetime, cv2

IMAGES_PATH = os.getenv("HOME") + "/data/images/"

class SaveFilesResult(IntEnum):
	SUCCESS = 0
	PARTIAL_SUCCESS = 1
	FAILURE = 2

class AImagePublisher(Node):

	def __init__(self):
		super().__init__('camera_publisher')

		self.declare_parameter('queue_size', 20, ParameterDescriptor(description="Max number of images stored in memory."))
		self.declare_parameter('resolution', [800, 600], ParameterDescriptor(description="Image resolution: [width, height]"), True)
		self.declare_parameter('interval', 0.5, ParameterDescriptor(description="Interval during each image"))
		self.declare_parameter('compression', 95, ParameterDescriptor(description="Compression level [0-100]"))
		self.declare_parameter('preview', True, ParameterDescriptor(description="Enable/Disable preview. Only before start"), True)
		self.update_parameters()

		self.img_queue = deque(maxlen=self.queue_size)
		self.bridge = CvBridge()
		self.count = 0

		self.init_camera()

		self.pub = self.create_publisher(Image, 'images', 10)
		self.save_service = self.create_service(SaveImages, 'save_images', self.save_files)
		self.timer = self.create_timer(0.2, self.routine)
		self.get_logger().info('Camera publisher node started')

	@abstractmethod
	def init_camera(self):
		pass

	@abstractmethod
	def capture(self):
		pass

	def save_files(self, request: SaveImages.Request, response : SaveImages.Response) -> SaveImages.Response:
		try:

			if (len(self.img_queue) == 0):
				raise Exception("no image available")
			now = datetime.datetime.now()
			delta: datetime.datetime = now.timestamp() - self.img_queue[0][0].timestamp()
			if (delta >= request.time):
				response.result = SaveFilesResult.SUCCESS
			else:
				response.result = SaveFilesResult.PARTIAL_SUCCESS

			for i in range(0, len(self.img_queue)):
				(img_time, img_data) = self.img_queue[i]
				if (now.timestamp() - img_time.timestamp() > request.time):
					continue
				path: str = f"{request.path}/{img_time.strftime('%m-%d_%H-%M-%S-%f')}.jpg"
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
			self.pub.publish(self.bridge.cv2_to_imgmsg(self.img_queue[-1][1], encoding))
			self.get_logger().info(f"Published image: {self.img_queue[-1][1].nbytes / 1024 / 1024:.2f}MB")

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
			raise e