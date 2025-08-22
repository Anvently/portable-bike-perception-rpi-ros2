#!/usr/bin/env python3
#!/usr/bin/env python3
# Created on Tue Aug 20 2025
# Updated on Tue Aug 20 2025
#
# This file is part of Cyclosafe
# Copyright (c) 2025 Nicolas Pirard @Anvently
#
# This software is governed by the CeCILL license under French law and
# abiding by the rules of distribution of free software. You can use,
# modify and/or redistribute the software under the terms of the CeCILL
# license as circulated by CEA, CNRS and INRIA at:
# https://cecill.info/licences/Licence_CeCILL-B_V1-en.html

"""
ROS Bag Data Exporter

This script exports data from ROS bags to CSV files and extracts images.
Supports LaserScan, CompressedImage, and NavSatInfo message types.
"""

import os
import sys
import argparse
import csv
import json
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any, Optional
import numpy as np
from concurrent.futures import ThreadPoolExecutor, as_completed

from rich.console import Console
from rich.progress import Progress, TaskID
from rich.logging import RichHandler
import logging

try:
	import rosbag2_py
	from rclpy.serialization import deserialize_message
	from rosidl_runtime_py.utilities import get_message
	import cv2
except ImportError as e:
	print(f"Missing required dependencies: {e}")
	print("Please install: pip install rosbag2-py rclpy rosidl-runtime-py opencv-python")
	sys.exit(1)

# Setup logging with Rich
console = Console()
logging.basicConfig(
	level=logging.INFO,
	format="%(message)s",
	datefmt="[%X]",
	handlers=[RichHandler(console=console, rich_tracebacks=True)]
)
logger = logging.getLogger(__name__)


class RosBagExporter:
	"""Export ROS bag data to CSV files and extract images."""
	
	def __init__(self, bag_path: str, output_path: Optional[str] = None, max_workers=4):
		"""
		Initialize the exporter.
		
		Args:
			bag_path: Path to the directory containing ROS bag files
			output_path: Output directory path (defaults to bag_path/export)
		"""
		self.bag_path = Path(bag_path)
		self.output_path = Path(output_path) if output_path else self.bag_path / "export"
		self.images_path = self.output_path / "images"
		self.executor = ThreadPoolExecutor(max_workers=max_workers)
		self.image_futures = []  # stocker les tâches asynchrones

		# Create output directories
		self.output_path.mkdir(parents=True, exist_ok=True)
		self.images_path.mkdir(parents=True, exist_ok=True)
		
		# Statistics
		self.stats = {
			"lidar_scans": 0,
			"gps_messages": 0,
			"images": 0,
			"errors": 0
		}
		
		# CSV writers
		self.csv_writers: Dict[str, Any] = {}
		self.csv_files: Dict[str, Any] = {}
		
	def __enter__(self):
		"""Context manager entry."""
		return self
		
	def __exit__(self, exc_type, exc_val, exc_tb):
		"""Context manager exit - close all files."""
		self.close_files()
		
	def close_files(self):
		"""Close all open CSV files."""
		for future in as_completed(self.image_futures):
			try:
				future.result()
			except Exception as e:
				logger.error(f"Image export task failed: {e}")
				self.stats["errors"] += 1
		self.executor.shutdown(wait=True)

		for writer in self.csv_files.values():
			if hasattr(writer, 'close'):
				writer.close()
		self.csv_files.clear()
		self.csv_writers.clear()
		
	def timestamp_to_datetime(self, timestamp_sec: float) -> str:
		"""Convert timestamp to datetime string."""
		return datetime.fromtimestamp(timestamp_sec).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
		
	def get_csv_writer(self, topic_name: str, message_type: str, headers: List[str]):
		"""Get or create CSV writer for a topic."""
		key = f"{topic_name}_{message_type}"
		
		if key not in self.csv_writers:
			filename = f"{topic_name.replace('/', '_').strip('_')}_{message_type}.csv"
			filepath = self.output_path / filename
			
			csv_file = open(filepath, 'w', newline='', encoding='utf-8')
			writer = csv.writer(csv_file)
			writer.writerow(headers)
			
			self.csv_files[key] = csv_file
			self.csv_writers[key] = writer
			
		return self.csv_writers[key]
		
	def export_laser_scan(self, msg, timestamp_sec: float, topic_name: str):
		"""Export LaserScan message to CSV."""
		try:
			headers = [
				'timestamp_sec', 'datetime', 'frame_id', 'scan_time', 'time_increment',
				'angle_min', 'angle_max', 'angle_increment', 'range_min', 'range_max',
				'num_points', 'ranges_array', 'intensities_array'
			]
			
			writer = self.get_csv_writer(topic_name, 'LaserScan', headers)
			
			ranges_array = np.array(msg.ranges, dtype=np.float64)
			intensities_array = np.array(msg.intensities, dtype=np.uint32)
			
			ranges_inf = np.isinf(ranges_array)
			ranges_nan = np.isnan(ranges_array)
			
			ranges_clean = np.where(ranges_inf | ranges_nan, ranges_array, 
								np.round(ranges_array, 3))
			
			ranges_list = ranges_clean.tolist()
			intensities_list = intensities_array.tolist()
			
			ranges_str = json.dumps(ranges_list)
			intensities_str = json.dumps(intensities_list)
			
			row = [
				timestamp_sec,
				self.timestamp_to_datetime(timestamp_sec),
				msg.header.frame_id,
				msg.scan_time,
				msg.time_increment,
				msg.angle_min,
				msg.angle_max,
				msg.angle_increment,
				msg.range_min,
				msg.range_max,
				len(msg.ranges),
				ranges_str,
				intensities_str
			]
			
			writer.writerow(row)
			self.stats["lidar_scans"] += 1
			
		except Exception as e:
			logger.error(f"Error exporting LaserScan: {e}")
			self.stats["errors"] += 1
			
	def export_nav_sat_info(self, msg, timestamp_sec: float, topic_name: str):
		"""Export NavSatInfo message to CSV."""
		try:
			headers = [
				'header_timestamp_sec', 'datetime', 'message_type', 'latitude', 'longitude',
				'altitude', 'status', 'service', 'frame_id'
			]
			
			writer = self.get_csv_writer(topic_name, 'NavSatInfo', headers)
			
			# Convert header timestamp
			header_timestamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
			
			row = [
				header_timestamp_sec,
				self.timestamp_to_datetime(timestamp_sec),
				'NavSatInfo',
				msg.latitude,
				msg.longitude,
				msg.altitude,
				msg.status.status,
				msg.status.service,
				msg.header.frame_id,
			]
			
			writer.writerow(row)
			self.stats["gps_messages"] += 1
			
		except Exception as e:
			logger.error(f"Error exporting NavSatInfo: {e}")
			self.stats["errors"] += 1
			
	def export_compressed_image(self, msg, msg_type, timestamp_sec: float):
		"""Export CompressedImage: metadata sync, image async."""
		dt = datetime.fromtimestamp(timestamp_sec)
		filename = dt.strftime("%Y%m%d-%H%M%S") + f"_{int((timestamp_sec % 1) * 1000):03d}{".jpeg" if "CompressedImage" in msg_type else ".png"}"
		filepath = self.images_path / filename
		
		writer = self.get_csv_writer('images', 'metadata', ['nom', 'date_heure', 'timestamp_unix'])
		writer.writerow([filename, self.timestamp_to_datetime(timestamp_sec), timestamp_sec])
		self.stats["images"] += 1
		
		def task():
			try:
				np_arr = np.frombuffer(msg.data, np.uint8)
				
				if 'CompressedImage' in msg_type:
					cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
				else:
					height = msg.height
					width = msg.width
					encoding = msg.encoding
					
					if encoding in ['rgb8', 'bgr8']:
						channels = 3
						dtype = np.uint8
					elif encoding in ['rgb16', 'bgr16']:
						channels = 3
						dtype = np.uint16
					elif encoding in ['mono8']:
						channels = 1
						dtype = np.uint8
					elif encoding in ['mono16']:
						channels = 1
						dtype = np.uint16
					else:
						logger.warning(f"Unsupported encoding: {encoding}")
						self.stats["errors"] += 1
						return
					
					if channels == 1:
						cv_img = np_arr.reshape((height, width))
					else:
						cv_img = np_arr.reshape((height, width, channels))
					
					cv_img = cv_img.astype(dtype)
				
				if cv_img is not None:
					cv2.imwrite(str(filepath), cv_img)
				else:
					logger.warning(f"Failed to decode image at timestamp {timestamp_sec}")
					self.stats["errors"] += 1
					
			except Exception as e:
				logger.error(f"Error exporting image: {e}")
				self.stats["errors"] += 1
		
		future = self.executor.submit(task)
		self.image_futures.append(future)

			
	def process_bag(self):
		"""Process the ROS bag and export data."""
		logger.info(f"Processing bag from: {self.bag_path}")
		
		# Setup bag reader
		storage_options = rosbag2_py.StorageOptions(
			uri=str(self.bag_path),
			storage_id='mcap'
		)
		converter_options = rosbag2_py.ConverterOptions(
			input_serialization_format='cdr',
			output_serialization_format='cdr'
		)
		
		reader = rosbag2_py.SequentialReader()
		reader.open(storage_options, converter_options)
		
		# Get topic metadata
		topic_types = reader.get_all_topics_and_types()
		topics_infos = reader.get_metadata().topics_with_message_count
		total_message = sum(
			topic.message_count
			for topic in topics_infos
			if topic.topic_metadata.type in ['sensor_msgs/msg/LaserScan', 'sensor_msgs/msg/CompressedImage', 'sensor_msgs/msg/Image', 'cyclosafe_interfaces/msg/NavSatInfo']
		)
		type_map = {topic.name: topic.type for topic in topic_types}
		logger.info(f"Found {len(type_map)} topics and {total_message} messages")
		
		# Process messages
		with Progress(console=console) as progress:
			task = progress.add_task("Processing messages...", total=total_message)
			
			message_count = 0
			while reader.has_next():
				topic_name, data, timestamp = reader.read_next()
				
				if topic_name not in type_map:
					continue
					
				msg_type = type_map[topic_name]
				timestamp_sec = timestamp / 1e9  # Convert nanoseconds to seconds
				
				try:
					# Deserialize message
					msg_class = get_message(msg_type)
					msg = deserialize_message(data, msg_class)
					
					# Export based on message type
					if 'LaserScan' in msg_type:
						self.export_laser_scan(msg, timestamp_sec, topic_name)
					elif 'NavSatInfo' in msg_type:
						self.export_nav_sat_info(msg, timestamp_sec, topic_name)
					elif 'Image' in msg_type:
						self.export_compressed_image(msg, msg_type, timestamp_sec)
						
				except Exception as e:
					logger.error(f"Error processing message from {topic_name}: {e}")
					self.stats["errors"] += 1
					
				message_count += 1
				if message_count % 100 == 0:
					progress.update(task, advance=100)
					
			progress.update(task, completed=message_count)
			
		reader.close()
		self.close_files()
		
		# Print statistics
		logger.info("Export completed")
		logger.info(f"LaserScan messages: {self.stats['lidar_scans']}")
		logger.info(f"GPS messages: {self.stats['gps_messages']}")
		logger.info(f"Images exported: {self.stats['images']}")
		logger.info(f"Errors: {self.stats['errors']}")
		logger.info(f"Output directory: {self.output_path}")


def main():
	"""Main function with profiling."""
	parser = argparse.ArgumentParser(
		description="Export ROS bag data to CSV files and extract images"
	)
	parser.add_argument(
		"bag_path",
		help="Path to the directory containing ROS bag files"
	)
	parser.add_argument(
		"-o", "--output",
		help="Output directory (default: bag_path/export)"
	)
	parser.add_argument(
		"-v", "--verbose",
		action="store_true",
		help="Enable verbose logging"
	)
	
	args = parser.parse_args()
	
	if args.verbose:
		logging.getLogger().setLevel(logging.DEBUG)
		
	# Validate input path
	bag_path = Path(args.bag_path)
	if not bag_path.exists():
		logger.error(f"Bag path does not exist: {bag_path}")
		sys.exit(1)
	
	
	# Process the bag with optional profiling
	try:
		with RosBagExporter(args.bag_path, args.output) as exporter:
			exporter.process_bag()
			logger.info("Export completed successfully")
	except Exception as e:
		logger.error(f"Export failed: {e}")
		sys.exit(1)

if __name__ == "__main__":
	main()