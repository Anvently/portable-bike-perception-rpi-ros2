#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Crerated on Tue Aug 05 2025
# Updated on Tue Aug 05 2025
#
# This file is part of Cyclosafe
# Copyright (c) 2025 Nicolas Pirard @Anvently
#
# This software is governed by the CeCILL license under French law and
# abiding by the rules of distribution of free software. You can use,
# modify and/or redistribute the software under the terms of the CeCILL
# license as circulated by CEA, CNRS and INRIA at:
# https://cecill.info/licences/Licence_CeCILL-B_V1-en.html

import os, time, sys, signal
import battery_monitor
import psutil
from battery_monitor import INA219
from gpiozero import Button, PWMLED

BTN_RST_GPIO = 16
LED_BATTERY_GPIO = 9
LED_BUZY_GPIO= 7
LED_SD_CARD_GPIO = 10

LED_BRIGHTNESS = float(os.getenv("LED_BRIGHTNESS", "1.0"))

button = Button(BTN_RST_GPIO, pull_up=True, bounce_time=0.1)

class BatteryException(Exception):
	pass

BUTTON_SHUTDOWN = 255
BATTERY_SHUTDOWN = 254

NBR_CELLS = 2
CHARGE_VOLTAGE = 4.2
MIN_VOLTAGE = 3

VOLTAGE_RANGE = CHARGE_VOLTAGE - MIN_VOLTAGE
BUS_CHARGE_VOLTAGE = NBR_CELLS * CHARGE_VOLTAGE
BATTERY_VOLTAGE_TRESHOLD = BUS_CHARGE_VOLTAGE - (NBR_CELLS * VOLTAGE_RANGE)

LOW_BATTERY_PERCENT = float(os.getenv("LOW_BATTERY_PERCENT", "0.2"))
LOW_BATTERY_TRESHOLD = BATTERY_VOLTAGE_TRESHOLD + (LOW_BATTERY_PERCENT * NBR_CELLS * VOLTAGE_RANGE)

LOW_STORAGE_TRESHOLD = int(os.getenv("LOW_STORAGE_TRESHOLD", "512"))

class GPIOController():
	def __init__(self):
		GPIOController.colors = [(255,0,0), (0,255,0), (0,0,255), (255,255,0), (0,255,255), (255,255,255)]
		self.color_index = 0
		self.gpio_state = {LED_BATTERY_GPIO: False, LED_BUZY_GPIO: False, LED_SD_CARD_GPIO: False}
		
		self.led_battery = PWMLED(LED_BATTERY_GPIO)
		self.led_busy = PWMLED(LED_BUZY_GPIO)
		self.led_sd = PWMLED(LED_SD_CARD_GPIO)
		
		for gpio in self.gpio_state:
			self.turn_off(gpio)
		self.pos = 0
		self.enable = True

		self.ina219 = INA219(1, addr=0x42)

	def turn_on(self, gpio):
		led = self._get_led(gpio)
		if led:
			led.value = LED_BRIGHTNESS
		self.gpio_state[gpio] = True

	def turn_off(self, gpio):
		led = self._get_led(gpio)
		if led:
			led.off()
		self.gpio_state[gpio] = False

	def _get_led(self, gpio):
		if gpio == LED_BATTERY_GPIO:
			return self.led_battery
		elif gpio == LED_BUZY_GPIO:
			return self.led_busy
		elif gpio == LED_SD_CARD_GPIO:
			return self.led_sd
		return None

	def toggle(self, gpio):
		if self.gpio_state[gpio]:
			self.turn_off(gpio)
		else:
			self.turn_on(gpio)

	def check_battery_state(self):
		if not self.ina219:
			return
		try:
			bus_voltage = self.ina219.get_bus_voltage_V()
		except:
			return
		if bus_voltage < BATTERY_VOLTAGE_TRESHOLD:
			raise BatteryException()
		elif bus_voltage < LOW_BATTERY_TRESHOLD:
			self.turn_on(LED_BATTERY_GPIO)

	def cleanup(self):
		self.turn_off(LED_BATTERY_GPIO)
		self.turn_off(LED_SD_CARD_GPIO)
		self.turn_on(LED_BUZY_GPIO)
		self.led_battery.close()
		self.led_sd.close()

	def routine(self, blink = True):
		shutdown_type = 0
		try:
			count = 0
			while not button.is_pressed:
				if count % 40 == 0:
					self.check_battery_state()
				if count % 120 == 0:
					self.check_sd_card()
				if blink:
					self.toggle(LED_BUZY_GPIO)
				time.sleep(0.25)
				count += 1
			print("Shutting down...")
			shutdown_type = BUTTON_SHUTDOWN
		except BatteryException:
			shutdown_type = BATTERY_SHUTDOWN
		
		self.cleanup()
		return shutdown_type

	def check_sd_card(self) -> bool:
		mb_available = psutil.disk_usage("/").free / 1024 / 1024
		print(mb_available, LOW_STORAGE_TRESHOLD)
		if (mb_available < LOW_STORAGE_TRESHOLD):
			self.turn_on(LED_SD_CARD_GPIO)
			return False
		self.turn_off(LED_SD_CARD_GPIO)
		return True

gpio_controler = GPIOController()

def signal_handler(signum, frame):
	gpio_controler.cleanup()
	sys.exit(0)

def main(args=None):
	global gpio_controler
	
	signal.signal(signal.SIGINT, signal_handler)
	signal.signal(signal.SIGTERM, signal_handler)
	shutdown_code = 0
	try:
		if gpio_controler.check_sd_card() == False:
			shutdown_code = gpio_controler.routine(blink=False)
		else:
			shutdown_code = gpio_controler.routine(blink=True)
	except:
		pass

	sys.exit(shutdown_code)

if __name__ == '__main__':
	main()
