# Created on Tue Aug 05 2025
# Updated on Tue Aug 05 2025
#
#  This file is part of Cyclosafe
# Copyright (c) 2025 Nicolas Pirard @Anvently
#
# This software is governed by the CeCILL license under French law and
# abiding by the rules of distribution of free software. You can use,
# modify and/or redistribute the software under the terms of the CeCILL
# license as circulated by CEA, CNRS and INRIA at:
# https://cecill.info/licences/Licence_CeCILL-B_V1-en.html


from PyQt5.QtCore import  pyqtSignal, QThread
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import os, time
from rclpy.time import Time, Duration
from typing import Dict

class BagInfo:
	"""Class to extract and store bag information."""
	def __init__(self, bag_path):
		self.bag_path = bag_path
		self.start_time = None
		self.end_time = None
		self.duration = 0
		self.topic_info = {}
		self.message_count = 0
		self.extract_info()
		
	def extract_info(self):
		if not (os.path.isfile(self.bag_path)):
			raise Exception("Invalid bag")
	
		reader = rosbag2_py.SequentialReader()
		
		# Check if the path is a directory (for ROS2 bags) or a file (like mcap)
		storage_id = 'sqlite3' if os.path.isdir(self.bag_path) else 'mcap'
		
		storage_options = rosbag2_py._storage.StorageOptions(uri=self.bag_path, storage_id=storage_id)
		converter_options = rosbag2_py._storage.ConverterOptions('', '')
		
		reader.open(storage_options, converter_options)
		
		# Get topic info
		self.topic_info = {}
		for topic_metadata in reader.get_all_topics_and_types():
			self.topic_info[topic_metadata.name] = type('TopicInfo', (), {
				'msg_type': topic_metadata.type,
				'message_count': 0  # Will count below
			})

		self.starting_time = reader.get_metadata().starting_time
			
		start_time = None
		end_time = None
		count = 0
		
		while reader.has_next():
			(topic, _, t) = reader.read_next()
			if start_time is None:
				start_time = t
			end_time = t
			
			# Increment message count for this topic
			if topic in self.topic_info:
				self.topic_info[topic].message_count += 1
			count += 1
			
		self.start_time = start_time
		self.end_time = end_time
		self.message_count = count
		
		if start_time is not None and end_time is not None:
			self.duration = (end_time - start_time) / 1e9  # ns to seconds


class BagReader(QThread):
	"""Thread to read and process bag files."""
	message_read = pyqtSignal(str, object, float)
	finished = pyqtSignal()
	# progress = pyqtSignal(int)
	
	def __init__(self, bag_path):
		super().__init__()
		self.bag_path = bag_path
		self.topics = []
		self.stop_flag = False
		self.pause_flag = False
		self.current_time = 0
		self.reader = None
		self.playback_speed = 1.0
		self.onMessageCallbacks: Dict[str, any] = {}

	def run(self):
		self.reader = rosbag2_py.SequentialReader()
		storage_options = rosbag2_py._storage.StorageOptions(uri=self.bag_path, storage_id='mcap')
		converter_options = rosbag2_py._storage.ConverterOptions('', '')
		self.reader.open(storage_options, converter_options)
		
		topic_types = {}
		for topic_metadata in self.reader.get_all_topics_and_types():
			topic_types[topic_metadata.name] = topic_metadata.type
		
		start_time = None
		while self.reader.has_next():
			if self.stop_flag:
				break
				
			while self.pause_flag:
				if self.stop_flag:
					break
				time.sleep(0.1)
			
			(topic, data, t) = self.reader.read_next()
			msg_type = get_message(topic_types[topic])
			msg = deserialize_message(data, msg_type)
			
			if start_time is None:
				start_time = t
				self.current_time = 0
			else:
				time_diff = (t - start_time) / 1e9  # nanoseconds to seconds
				if time_diff > self.current_time:
					sleep_time = (time_diff - self.current_time) / self.playback_speed
					if sleep_time > 0:
						self.current_time = time_diff
						if (sleep_time < 1):
							time.sleep(sleep_time)
			
			self.message_read.emit(topic, msg, self.current_time)
		
		self.finished.emit()
	
	def seek(self, time_pos):
		if self.reader is not None:
			# desired_timestamp = int(time_pos * 1e9)  # seconds to nanoseconds
			self.current_time = time_pos
			time_start: Time = self.reader.get_metadata().starting_time
			desired_timestamp = time_start.nanoseconds + int(time_pos * 1e9)
			self.reader.seek(desired_timestamp)
	
	def set_playback_speed(self, speed):
		self.playback_speed = speed
	
	def pause(self):
		self.pause_flag = True
	
	def resume(self):
		self.pause_flag = False
	
	def stop(self):
		self.stop_flag = True