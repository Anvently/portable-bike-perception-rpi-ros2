import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import time
from rcl_interfaces.msg import ParameterDescriptor
import rclpy.qos
from sensor_msgs.msg import Range
from rclpy.time import Time
import pigpio

TRIGGER_GPIO = 6
ECHO_GPIO = 21
FOV = 3.0
SOUND_SPEED = 340 # meters/s

class SonarSR04Node(Node):

	def __init__(self):
		super().__init__('sonar_sr04')
		self.pending_measure = False

		self.declare_parameter("period", 0.1, ParameterDescriptor(description="interval between each measurement"))
		self.declare_parameter('start_time', 0.0, ParameterDescriptor(description="Time to be used as the beginning of the simulation. Float value of seconds since epoch."))

		self.period = self.get_parameter('period').get_parameter_value().double_value
		self.publisher = self.create_publisher(Range, 'range', 10)
		
		self.pi = pigpio.pi()
		
		self.pi.set_mode(TRIGGER_GPIO, pigpio.OUTPUT)
		self.pi.set_mode(ECHO_GPIO, pigpio.INPUT)
		self.pi.write(TRIGGER_GPIO, pigpio.LOW)
		
		self.timer = self.create_timer(self.period, self.measure)

	def measure(self):
		# Send trigger signal (pull trigger pin high for 10us)
		if self.pending_measure:
			self.get_logger().warning("Period is to fast ! Cannot trigger a measurement before ending the previous one")
			return
		self.pending_measure = True
		self.pi.write(TRIGGER_GPIO, pigpio.HIGH)
		time.sleep(0.00001)
		self.pi.write(TRIGGER_GPIO, pigpio.LOW)

		start_time = time.time()
		start_loop_time = start_time
		while (self.pi.read(ECHO_GPIO) == pigpio.LOW):
			start_time = time.time()
			if (start_time - start_loop_time > self.period):
				self.handle_time_out()
				return

		while (self.pi.read(ECHO_GPIO) == pigpio.HIGH):
			if (time.time() - start_time) > self.period:
				self.handle_time_out()
				return
			time.sleep(0.0001)

		stop_time = time.time()
		distance = ((stop_time - start_time) * SOUND_SPEED) / 2
		self.get_logger().debug(f"Measured d={distance}")
		self.publish(distance)
		self.pending_measure = False
		
		if self.timer.timer_period_ns != self.period * 1000 * 1000 * 1000:
			self.timer.timer_period_ns = self.period * 1000 * 1000 * 1000
			self.timer.reset()


	def handle_time_out(self):
		self.get_logger().warning("Timeout !")
		self.pending_measure = False
		self.timer.timer_period_ns = 10 * 1000 * 1000 * 1000

	def publish(self, distance: float):
		msg = Range()
		msg.range = distance
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.field_of_view = FOV
		msg.max_range = distance
		msg.min_range = distance
		self.publisher.publish(msg)

def main(args=None):
	try:
		rclpy.init(args=args)
		sonar_node = SonarSR04Node()
		rclpy.spin(sonar_node)
	except (KeyboardInterrupt, ExternalShutdownException):
		pass

if __name__ == "main":
	main()