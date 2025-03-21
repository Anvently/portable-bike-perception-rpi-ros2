import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from cyclosafe_interfaces.srv import SaveImages
import os
from datetime import datetime
from typing import Tuple, List, Dict, Deque
from collections import deque
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Range

"""
GPS:
0,lat,long,hdop
7,lat+1,long+1,hdop

Lidar:
0,

"""

class Datas:

	def __init__(self, fields: List[str], ttl: float, save_path: str):
		"""
			fields: each field has its dedicated deque
			ttl: time-to-live, lifespan of the data in their respective deque
			path: directory to which file will be stored
		"""
		self.start_time: datetime = datetime.now()
		self.fields: Dict[Deque[Tuple[float, any]]] = {}
		self.ttl = ttl
		for field in fields:
			self.fields[field] = deque()
		
	def add_measure(self, key: str, data: Tuple[datetime, any]):
		self.check_ttl(self, key)
		
		pass

	def check_ttl(self, key: str):
		time = datetime.now()
		container = self.fields[key]
		data: Tuple[float, any] = container[0]
		while time - data[0] > self.ttl and len(container): 
			container.popleft()
			data = container[0]

	def save(self):
		
		pass


class AggregatorNode(Node):
	def __init__(self):
		super().__init__('aggregator')

		self.future = None

		self.declare_parameter('cache_ttl', 30.0, ParameterDescriptor(description="Period (in seconds) during which every information from the sensor is retained by the aggregator"))
		self.declare_parameter('save_interval', 10.0, ParameterDescriptor(description="Interval between each write to out files"))
		self.declare_parameter('save_path', os.getenv("HOME") + "/data/")

		self._cache_ttl = self.get_parameter('cache_ttl').get_parameter_value().double_value
		self.save_interval = self.get_parameter('save_interval').get_parameter_value().double_value

		self._save_path = self.get_parameter('save_path').get_parameter_value().string_value

		time = datetime.now().strftime("%Y%m%d-%H%M%S")
		self.out_path = f"{self._save_path}/{time}/"
		os.mkdir(self.out_path, mode = 511)
		os.mkdir(f"{self.out_path}/images/", mode = 511)
		symlink_path = f"{self._save_path}/latest"
		if (os.path.islink(symlink_path)):
			os.remove(symlink_path)
		os.symlink(self.out_path, symlink_path, os.O_CREAT | os.O_DIRECTORY | os.O_NOFOLLOW)

		self.datas = Datas(['range', 'gps'], 20.0, self.out_path)

		self.gps_susbriber = self.create_subscription(NavSatFix, 'gps', self.gps_callback, 10)
		self.gps_susbriber = self.create_subscription(Range, 'range', self.range_callback, 10)

		self.save_timer = self.create_timer(self.save_interval, self.save_data)
		
		self.save_client = self.create_client(SaveImages, 'save_images')
		self.get_logger().info("Node started")

	@property
	def cache_ttl(self):
		return self._cache_ttl
	
	@property
	def cache_ttl(self, value: float):
		self._cache_ttl = value
		self.save_timer.timer_period_ns = value * 1000 * 1000 * 1000

	def gps_callback(self, messsage: NavSatFix):
		pass

	def range_callback(self, message: Range):
		pass

	def save_data(self):
		request = SaveImages.Request(time=int(self.save_interval), path=f"{self.out_path}/images/")
		if (self.future and self.future.done() == False):
			self.get_logger().error("No response from camera node. Is it running ?")
			self.future.cancel()
		self.future = self.save_client.call_async(request)

		def	_done_callback(future):
			if (not future or future.done() == False):
				return
			response: SaveImages.Response = future.result()
			if (response.result == SaveImages.Response.FAILURE):
				self.get_logger().error(f"Lost images from the the past {self.save_interval} seconds")
			elif (response.result == SaveImages.Response.PARTIAL_SUCCESS):
				self.get_logger().warning(f"Some images from the past {self.save_interval} will be missing")
		self.future.add_done_callback(_done_callback)
		


def main(args=None):
	try:
		rclpy.init(args=args)
		image_pubisher = AggregatorNode()
		rclpy.spin(image_pubisher)
	
	except (KeyboardInterrupt, ExternalShutdownException):
		pass

	finally:
		rclpy.shutdown()

if __name__ == '__main__':
	main()
