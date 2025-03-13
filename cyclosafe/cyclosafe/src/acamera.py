from rclpy.node import Node, ParameterDescriptor
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from collections import deque
from abc import abstractmethod

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
		self.timer = self.create_timer(0.2, self.routine)
		self.get_logger().info('Camera publisher node started')

	@abstractmethod
	def init_camera(self):
		pass

	@abstractmethod
	def capture(self):
		pass

	def publish(self, encoding="rgb8"):
		if (len(self.img_queue) > 0):
			self.pub.publish(self.bridge.cv2_to_imgmsg(self.img_queue[-1], encoding))
			self.get_logger().info(f"Published image: {self.img_queue[-1].nbytes / 1024 / 1024:.2f}MB")

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