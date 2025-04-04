import pigpio, sys
import os, time

BTN_RST_GPIO = 16
BTN_TEST_GPIO = 16
LED_R_GPIO = 7
LED_G_GPIO = 8
LED_B_GPIO = 9


pi = pigpio.pi()
stop = False

class GPIOController():
	
	"""
		Mutliple tasks:
			- monitor battery state
			- control state led
	"""

	def __init__(self):
		GPIOController.colors = [(255,0,0), (0,255,0), (0,0,255), (255,255,0), (0,255,255), (255,255,255)]
		self.color_index = 0
		pi.set_mode(LED_R_GPIO, pigpio.OUTPUT)
		pi.set_mode(LED_G_GPIO, pigpio.OUTPUT)
		pi.set_mode(LED_B_GPIO, pigpio.OUTPUT)
		pi.set_PWM_range(LED_R_GPIO, 255)
		pi.set_PWM_range(LED_G_GPIO, 255)
		pi.set_PWM_range(LED_B_GPIO, 255)
		self.set_rgb(0,0,0)
		self.pos = 0
		self.enable = True

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

	def routine(self):
		while stop == False:
			if self.enable:
				self.set_rgb(0, 0, 0)
				self.enable = False
			else:
				self.set_rgb(0, 127, 0)
				self.enable = True
			time.sleep(0.5)
		gpio_controler.set_rgb(127, 127, 0)
		sys.exit(1)
		
gpio_controler = GPIOController()

def host_shutdown(GPIO, level, tick):
	"""Shutdown host computer."""
	print("Shutting down...")
	global stop
	gpio_controler.set_rgb(127, 127, 0)
	stop = True

def main(args=None):
	global gpio_controler
	pi.set_mode(BTN_RST_GPIO, pigpio.INPUT)
	pi.callback(BTN_RST_GPIO, pigpio.FALLING_EDGE, host_shutdown)
	gpio_controler.routine()
	
	sys.exit(0)

if __name__ == '__main__':
	main()
