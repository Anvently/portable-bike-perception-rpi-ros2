import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
import pigpio

BTN_RST_GPIO = 16
BTN_TEST_GPIO = 25
LED_R_GPIO = 7
LED_G_GPIO = 8
LED_B_GPIO = 9


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
		self.duty_cycle = 0
		pi.callback(BTN_TEST_GPIO, pigpio.FALLING_EDGE, self.button_callback)
		pi.set_mode(BTN_TEST_GPIO, pigpio.INPUT)
		pi.set_PWM_dutycycle(LED_G_GPIO, self.duty_cycle)
		pi.set_PWM_range(LED_G_GPIO, 255)
		self.create_timer(0, self.routine)
		self.get_logger().info('gpio node started')
		self.enable = True

	def button_callback(self, GPIO, level, tick):
		self.enable = not self.enable
		self.get_logger().info(f"button pushed: {GPIO}, {level}, {tick}")

	def routine(self):
		if self.enable:
			self.duty_cycle = (self.duty_cycle + 1) % 255
		else:
			self.duty_cycle = 0
		pi.set_PWM_dutycycle(LED_G_GPIO, self.duty_cycle)


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
