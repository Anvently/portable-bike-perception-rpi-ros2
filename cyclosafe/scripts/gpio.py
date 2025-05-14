import pigpio, sys
import os, time
import battery_monitor
from battery_monitor import INA219

BTN_RST_GPIO = 16
BTN_TEST_GPIO = 16
# LED_R_GPIO = 7
LED_G_GPIO = 8
# LED_B_GPIO = 9


pi = pigpio.pi()
button_pressed = False

class BatteryException(Exception):
	pass

BUTTON_SHUTDOWN = 255
BATTERY_SHUTDOWN = 254

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
		pi.set_mode(LED_G_GPIO, pigpio.OUTPUT)
		pi.set_PWM_range(LED_G_GPIO, 255)
		self.set_rgb(0,0,0)
		self.pos = 0
		self.enable = True

		self.ina219 = INA219(1, 0x42)
		ina219_reading_mA = 1000
		ext_meter_reading_mA = 1000
		while not self.ina219.begin():
			time.sleep(2)
		self.ina219.linear_cal(ina219_reading_mA, ext_meter_reading_mA)

	def set_rgb(self, r, g, b):
		pi.set_PWM_dutycycle(LED_G_GPIO, g)

	def blink(self):
		if self.enable:
			self.set_rgb(0, 0, 0)
			self.enable = False
		else:
			self.set_rgb(0, 255, 0)
			self.enable = True

	def check_battery_state(self):
		if not self.ina219:
			return
		bus_voltage = self.ina219.get_bus_voltage_V()
		if bus_voltage < self.ina219.min_voltage:
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
	pi.set_pull_up_down(BTN_RST_GPIO, pigpio.PUD_UP)
	pi.set_mode(BTN_RST_GPIO, pigpio.INPUT)
	pi.callback(BTN_RST_GPIO, pigpio.FALLING_EDGE, host_shutdown)
	gpio_controler.routine()
	
	sys.exit(0)

if __name__ == '__main__':
	main()
