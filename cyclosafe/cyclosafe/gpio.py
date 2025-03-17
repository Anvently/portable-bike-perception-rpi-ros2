import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.msg import ParameterDescriptor
import pigpio

BTN_RST_GPIO = 16
LED_R_GPIO = 7
LED_G_GPIO = 8
LED_B_GPIO = 9


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
		pi.set_PWM_dutycycle(LED_G_GPIO, self.duty_cycle)
		self.create_timer(0.1, self.routine)
		self.get_logger('gpio node started')
		
	def routine(self):
		self.duty_cycle = (self.duty_cycle + 1) % 255
		pi.set_PWM_dutycycle(LED_G_GPIO, self.duty_cycle)


def main(args=None):
	try:
		rclpy.init(args=args)
		gpio_controler = GPIOController()
		rclpy.spin(gpio_controler)
	
	except (KeyboardInterrupt, ExternalShutdownException):
		pass

if __name__ == '__main__':
	pi = pigpio.pi()
	pi.callback(BTN_RST_GPIO, pigpio.FALLING_EDGE, host_shutdown)
	main()
