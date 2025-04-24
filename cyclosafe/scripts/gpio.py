import pigpio, sys
import os, time
from cyclosafe.battery_monitor import INA219

BTN_RST_GPIO = 16
BTN_TEST_GPIO = 16
LED_R_GPIO = 7
LED_G_GPIO = 8
LED_B_GPIO = 9


pi = pigpio.pi()
button_pressed = False

class BatteryException(Exception):
	pass

BUTTON_SHUTDOWN = 1
BATTERY_SHUTDOWN = 2

"""
 Total voltage	= nbr_cell * charge_voltage
				= 2 * 4.2 = 8.4

 Range_per_cell	= charge_voltage - min_voltage
				= 4.2 - 3 = 1.2
 Min_voltage	= Total voltage - (nbr_cell * range_per_cell)
				= 8.4 - (2 * 1.2) = 6
"""
NBR_CELLS = 2
CHARGE_VOLTAGE = 4.2
MIN_VOLTAGE = 3
VOLTAGE_RANGE = CHARGE_VOLTAGE - MIN_VOLTAGE
BUS_CHARGE_VOLTAGE = NBR_CELLS * CHARGE_VOLTAGE
BATTERY_VOLTAGE_TRESHOLD = BUS_CHARGE_VOLTAGE - (NBR_CELLS * VOLTAGE_RANGE)

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

		self.ina219 = INA219(addr=0x42)

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

	def blink(self):
		if self.enable:
			self.set_rgb(0, 0, 0)
			self.enable = False
		else:
			self.set_rgb(0, 127, 0)
			self.enable = True

	def check_battery_state(self):
		bus_voltage = self.ina219.getBusVoltage_V()
		if bus_voltage < BATTERY_VOLTAGE_TRESHOLD:
			raise BatteryException()

	def routine(self):
		shutdown_type = 0
		try:
			count = 0
			while button_pressed == False:
				if count % 20 == 0: # 20 * 0.5 = 10, every 10s
					self.check_battery_state()
				self.blink()
				time.sleep(0.5)
				count += 1
			shutdown_type = BUTTON_SHUTDOWN
		except BatteryException:
			shutdown_type = BATTERY_SHUTDOWN
		gpio_controler.set_rgb(127, 127, 0)
		sys.exit(shutdown_type)
		
gpio_controler = GPIOController()

def host_shutdown(GPIO, level, tick):
	"""Shutdown host computer."""
	print("Shutting down...")
	global button_pressed
	gpio_controler.set_rgb(127, 127, 0)
	button_pressed = True

def main(args=None):
	global gpio_controler
	pi.set_mode(BTN_RST_GPIO, pigpio.INPUT)
	pi.callback(BTN_RST_GPIO, pigpio.FALLING_EDGE, host_shutdown)
	gpio_controler.routine()
	
	sys.exit(0)

if __name__ == '__main__':
	main()
