import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
import pigpio
import os, time

BTN_RST_GPIO = 16
BTN_TEST_GPIO = 16
LED_R_GPIO = 7
LED_G_GPIO = 8
LED_B_GPIO = 9


pi = pigpio.pi()

def host_shutdown(GPIO, level, tick):
	"""Shutdown host computer."""
	print("Shutting down...")
	time.sleep(0.5)
	rclpy.shutdown()
	os.system('sudo halt')

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
		pi.callback(BTN_TEST_GPIO, pigpio.FALLING_EDGE, self.button_callback)
		self.create_timer(0.5, self.routine)
		self.pos = 0
		self.enable = True
		self.get_logger().info('gpio node started')

	def set_rgb(self, r, g, b):
		pi.set_PWM_dutycycle(LED_R_GPIO, r)
		pi.set_PWM_dutycycle(LED_G_GPIO, g)
		pi.set_PWM_dutycycle(LED_B_GPIO, b)

	def wheel(self):
		self.pos = (self.pos + 1) % 255
		if (self.pos < 85):
			self.set_rgb(255 - self.pos * 3, 0, self.pos * 3)
		elif (self.pos < 170):
			pos = self.pos - 85
			self.set_rgb(0, pos * 3, 255 - pos * 3)
		else:
			pos = self.pos - 170
			self.set_rgb(pos * 3, pos * 3, 0)

	def button_callback(self, GPIO, level, tick):
		# self.color_index = (self.color_index + 1) % len(GPIOController.colors)
		# self.set_rgb(*GPIOController.colors[self.color_index])
		self.get_logger().info(f"button pushed: {GPIO}, {level}, {tick}")
		rclpy.shutdown()	
		self.enable = not self.enable

	def routine(self):
		if self.enable:
			self.set_rgb(0, 0, 0)
			self.enable = False
		else:
			self.set_rgb(0, 255, 0)
			self.enable = True

def main(args=None):
	try:
		pi.set_mode(BTN_RST_GPIO, pigpio.INPUT)
		pi.callback(BTN_RST_GPIO, pigpio.FALLING_EDGE, host_shutdown)
		rclpy.init(args=args)
		gpio_controler = GPIOController()
		rclpy.spin(gpio_controler)
	
	except (KeyboardInterrupt):
		pass
	except (ExternalShutdownException):
		print("external shutdopwn")
		sys.exit(1)

if __name__ == '__main__':
	main()
