#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pigpio, sys
import os, time
import battery_monitor
import psutil
from battery_monitor import INA219

BTN_RST_GPIO = 16
LED_BATTERY_GPIO = 9
LED_BUZY_GPIO= 7
LED_SD_CARD_GPIO = 10
# LED_B_GPIO = 9

# Led are turned off via PWM with [0;255] scale.
# Dutycycle is set to (LED_BRIGHTNESS * 255)
# So full brightness is achieved with LED_BRIGHTNESS = 1.0 and half brightness with LED_BRIGHTNESS = 0.5
LED_BRIGHTNESS = int(os.getenv("LED_BRIGHTNESS", "1.0")) # Full brightness

pi = pigpio.pi()
button_pressed = False

class BatteryException(Exception):
	pass

# Valeurs de retours

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
# Nombre d'accus de la batterie
NBR_CELLS = 2

# Tension de charge pour un accu
CHARGE_VOLTAGE = 4.2
# Tension minimale pour un accu 
# (techniquement le pallier limite est à 2.5V)
# (dans les faits les sécurités hardware, cad l'extinction brutale, se mettent en place à 2.8V)
MIN_VOLTAGE = 3 

VOLTAGE_RANGE = CHARGE_VOLTAGE - MIN_VOLTAGE
BUS_CHARGE_VOLTAGE = NBR_CELLS * CHARGE_VOLTAGE
BATTERY_VOLTAGE_TRESHOLD = BUS_CHARGE_VOLTAGE - (NBR_CELLS * VOLTAGE_RANGE)

# Voltage treshold when battery is less than 20%
LOW_BATTERY_POURCENT = int(os.getenv("LOW_BATTERY_POURCENT", "0.2"))
LOW_BATTERY_TRESHOLD = BATTERY_VOLTAGE_TRESHOLD + (LOW_BATTERY_POURCENT * NBR_CELLS * VOLTAGE_RANGE)

# When free storage is less than this threshold (in MB), the SD_CARD led will be turned on.
LOW_STORAGE_TRESHOLD = int(os.getenv("LOW_STORAGE_TRESHOLD", "512")) # (512MB)

class GPIOController():
	
	"""
		Mutliple tasks:
			- monitor battery state
			- control state led
	"""

	def __init__(self):
		GPIOController.colors = [(255,0,0), (0,255,0), (0,0,255), (255,255,0), (0,255,255), (255,255,255)]
		self.color_index = 0
		self.gpio_state = {LED_BATTERY_GPIO: False, LED_BUZY_GPIO: False, LED_SD_CARD_GPIO: False}
		for gpio in self.gpio_state:
			pi.set_mode(gpio, pigpio.OUTPUT)
			pi.set_PWM_range(gpio, 255)
			self.turn_off(gpio)
			self.pos = 0
		self.enable = True

		self.ina219 = None
		# self.ina219 = INA219(addr=0x42)

	def turn_on(self, gpio):
		pi.set_PWM_dutycycle(gpio, int(LED_BRIGHTNESS * 255))
		self.gpio_state[gpio] = True

	def turn_off(self, gpio):
		pi.set_PWM_dutycycle(gpio, 0)
		self.gpio_state[gpio] = False

	def toggle(self, gpio):
		if self.gpio_state[gpio]:
			self.turn_off(gpio)
		else:
			self.turn_on(gpio)

	def check_battery_state(self):
		if not self.ina219:
			return
		bus_voltage = self.ina219.getBusVoltage_V()
		if bus_voltage < BATTERY_VOLTAGE_TRESHOLD:
			raise BatteryException()
		elif bus_voltage < LOW_BATTERY_TRESHOLD:
			self.turn_on(LED_BATTERY_GPIO)

	def routine(self):
		shutdown_type = 0
		try:
			count = 0
			while button_pressed == False:
				if count % 20 == 0: # 20 * 0.5 = 10, every 10s
					self.check_battery_state()
				self.toggle(LED_BUZY_GPIO)
				time.sleep(0.5)
				count += 1
			shutdown_type = BUTTON_SHUTDOWN
		except BatteryException:
			shutdown_type = BATTERY_SHUTDOWN
		gpio_controler.turn_on(LED_BUZY_GPIO)
		sys.exit(shutdown_type)
		

	def check_sd_card(self) -> bool:
		mb_available = psutil.disk_usage("/").free / 1024 / 1024
		return (mb_available < LOW_STORAGE_TRESHOLD)


gpio_controler = GPIOController()

def host_shutdown(GPIO, level, tick):
	"""Shutdown host computer."""
	print("Shutting down...")
	global button_pressed
	gpio_controler.turn_on(LED_BUZY_GPIO)
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
