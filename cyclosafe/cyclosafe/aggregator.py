import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
from cyclosafe_interfaces.srv import SaveImages
import os, datetime

class AggregatorNode(Node):
	def __init__(self):
		super().__init__('aggregator')

		self.declare_parameter('cache_ttl', 30.0, ParameterDescriptor(description="Period (in seconds) during which every information from the sensor is retained by the aggregator"))
		self.declare_parameter('save_interval', 10.0, ParameterDescriptor(description="Interval during each write to the sd card"))
		self.declare_parameter('save_path', os.getenv("HOME") + "/data/")

		self._cache_ttl = self.get_parameter('cache_ttl').get_parameter_value().double_value
		self.save_interval = self.get_parameter('save_interval').get_parameter_value().double_value

		self._save_path = self.get_parameter('save_path').get_parameter_value().string_value

		time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
		self.out_path = f"{self._save_path}/{time}/"
		os.mkdir(self.out_path, mode = 511)
		symlink_path = f"{self._save_path}/latest"
		if (os.path.islink(symlink_path)):
			os.remove(symlink_path)
		os.symlink(self.out_path, symlink_path, os.O_CREAT | os.O_DIRECTORY | os.O_NOFOLLOW)

		self.save_timer = self.create_timer(self.save_interval, self.save_data)
		
		self.save_client = self.create_client(SaveImages, 'save_images')

	@property
	def cache_ttl(self):
		return self._cache_ttl
	
	@property
	def cache_ttl(self, value: float):
		self._cache_ttl = value
		self.save_timer.timer_period_ns = value * 1000 * 1000 * 1000

	def save_data(self):
		request = SaveImages.Request(time=int(self.save_interval), path=f"{self.out_path}/images/")
		future = self.save_client.call_async(request)
		rclpy.spin_until_future_complete(self, future, timeout_sec=5)
		if (future.done() == False):
			self.get_logger().error("No response from camera node. Is it running ?")
			return
		response: SaveImages.Response = future.result()
		if (response.result == SaveImages.Response.FAILURE):
			self.get_logger().error(f"Lost images from the the past {self.save_interval} seconds")
		elif (response.result == SaveImages.Response.PARTIAL_SUCCESS):
			self.get_logger().warning(f"Some images from the past {self.save_interval} will be missing")

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
