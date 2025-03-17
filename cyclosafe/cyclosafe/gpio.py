import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
import pigpio

BTN_RST_GPIO = 16
BTN_TEST_GPIO = 25
LED_R_GPIO = 7
LED_G_GPIO = 8
LED_B_GPIO = 20


pi = pigpio.pi()

def host_shutdown():
	"""Shutdown host computer."""
	pass


class GPIOController(Node):
	
	"""
		Mutliple tasks:
			- monitor battery state
			- control state led
	"""

	def __init__(self):
		super().__init__('gpio_controller')
		GPIOController.colors = [(255,0,0), (0,255,0), (0,0,255), (255,255,0), (0,255,255), (255,255,255)]
		self.color_index = 0
		pi.set_mode(LED_R_GPIO, pigpio.OUTPUT)
		pi.set_mode(LED_G_GPIO, pigpio.OUTPUT)
		pi.set_mode(LED_B_GPIO, pigpio.OUTPUT)
		pi.set_PWM_range(LED_R_GPIO, 255)
		pi.set_PWM_range(LED_G_GPIO, 255)
		pi.set_PWM_range(LED_B_GPIO, 255)
		self.set_rgb(0,0,0)
		pi.set_mode(BTN_TEST_GPIO, pigpio.INPUT)
		pi.callback(BTN_TEST_GPIO, pigpio.FALLING_EDGE, self.button_callback)
		self.create_timer(0, self.routine)
		self.pos = 0
		self.enable = True
		self.get_logger().info('gpio node started')

	def set_rgb(self, r, g, b):
		pi.set_PWM_dutycycle(LED_R_GPIO, r)
		pi.set_PWM_dutycycle(LED_G_GPIO, g)
		pi.set_PWM_dutycycle(LED_B_GPIO, b)

	def wheel(self):
		pos = 255 - pos
		if (pos < 85):
			self.set_rgb(255 - pos * 3, 0, pos * 3)
		elif (pos < 170):
			pos = pos - 85
			self.set_rgb(0, pos * 3, 255 - pos * 3)
		else:
			pos = pos - 170
			self.set_rgb(pos * 3, 255 - pos * 3, 0)

	def button_callback(self, GPIO, level, tick):
		# self.color_index = (self.color_index + 1) % len(GPIOController.colors)
		# self.set_rgb(*GPIOController.colors[self.color_index])
		exit(1)
		self.enable = not self.enable
		self.get_logger().info(f"button pushed: {GPIO}, {level}, {tick}")

	def routine(self):
		if self.enable:
			self.pos = (self.pos + 1) % 255
			self.wheel()

def main(args=None):
	try:
		rclpy.init(args=args)
		gpio_controler = GPIOController()
		rclpy.spin(gpio_controler)
	
	except (KeyboardInterrupt, ExternalShutdownException):
		pass

if __name__ == '__main__':
	pi.callback(BTN_RST_GPIO, pigpio.FALLING_EDGE, host_shutdown)
	main()
