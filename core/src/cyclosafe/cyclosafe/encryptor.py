#!/usr/bin/env python3
# Created on Tue Aug 19 2025
# Updated on Tue Aug 19 2025
#
# This file is part of Cyclosafe
# Copyright (c) 2025 Nicolas Pirard @Anvently
#
# This software is governed by the CeCILL license under French law and
# abiding by the rules of distribution of free software. You can use,
# modify and/or redistribute the software under the terms of the CeCILL
# license as circulated by CEA, CNRS and INRIA at:
# https://cecill.info/licences/Licence_CeCILL-B_V1-en.html

import os
import time
from pathlib import Path
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
import secrets
import json
import zstandard as zstd
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

"""
Simplified ROS2 node: monitors a directory and encrypts/compresses new bag files.
- Removed all publishers, services, and diagnostics
- Keeps graceful shutdown with final processing
"""


class ROSBagCryptoHandler(FileSystemEventHandler):
	"""Event handler to monitor and encrypt ROS bag files"""

	def __init__(self, public_key_path: str, watch_dir: str, ros_node: Node):
		self.ros_node = ros_node  # set first; used by _load_public_key
		self.public_key = self._load_public_key(public_key_path)
		self.watch_dir = Path(watch_dir)

		# File types to watch
		self.bag_extensions = {'.db3', '.bag', '.mcap'}

		# Track files currently being processed (to avoid duplicates)
		self.processing_files = set()

		# Shutdown flag
		self.shutdown_requested = False

		# Stats (internal only)
		self.stats = {
			'files_encrypted': 0,
			'files_failed': 0,
			'total_bytes_processed': 0,
			'start_time': time.time(),
		}

	def request_shutdown(self):
		self.shutdown_requested = True
		self.ros_node.get_logger().info("Shutdown requested, will process remaining files…")

	def _load_public_key(self, public_key_path: str):
		try:
			with open(public_key_path, 'rb') as key_file:
				public_key = serialization.load_pem_public_key(key_file.read())
			self.ros_node.get_logger().info("Public key successfully loaded")
			return public_key
		except Exception as e:
			self.ros_node.get_logger().error(f"Error while loading public key: {e}")
			raise

	def _is_bag_file(self, file_path: str) -> bool:
		return Path(file_path).suffix.lower() in self.bag_extensions

	def _is_file_complete(self, file_path: str, stability_time: float = 15) -> bool:
		"""Check if file size is stable for `stability_time` seconds."""
		try:
			size1 = os.path.getsize(file_path)
			time.sleep(stability_time)
			size2 = os.path.getsize(file_path)
			return size1 == size2 and size1 > 0
		except (OSError, FileNotFoundError):
			return False

	def _wait_for_file_stability(self, file_path: Path, max_wait_time: float = 30) -> bool:
		self.ros_node.get_logger().info(f"Waiting for file stability: {file_path}")
		start_time = time.time()
		while time.time() - start_time < max_wait_time:
			if self._is_file_complete(str(file_path), stability_time=3):
				self.ros_node.get_logger().info(f"File is stable: {file_path}")
				return True
			time.sleep(1)
		self.ros_node.get_logger().warning(f"File did not stabilize within {max_wait_time}s: {file_path}")
		return False

	def _find_unencrypted_bags(self):
		unencrypted_bags = []
		for file_path in self.watch_dir.rglob('*'):
			if (
				file_path.is_file()
				and self._is_bag_file(str(file_path))
				and not file_path.name.endswith('.enc')
			):
				unencrypted_bags.append(file_path)
		return unencrypted_bags

	def _secure_delete(self, file_path: Path) -> bool:
		try:
			os.remove(file_path)
			return True
		except Exception as e:
			self.ros_node.get_logger().warning(f"Unable to delete file: {e}")
			return False

	def _encrypt_file(self, file_path: str):
		"""Encrypt a file with hybrid RSA (OAEP) + AES-256-CBC with Zstandard compression."""
		file_path = Path(file_path)

		if file_path in self.processing_files:
			return
		self.processing_files.add(file_path)

		try:
			self.ros_node.get_logger().info(f"Starting encryption of: {file_path}")

			# Generate random AES key and IV
			aes_key = secrets.token_bytes(32)  # AES-256
			iv = secrets.token_bytes(16)

			# Encrypt AES key with RSA public key
			encrypted_aes_key = self.public_key.encrypt(
				aes_key,
				padding.OAEP(
					mgf=padding.MGF1(algorithm=hashes.SHA256()),
					algorithm=hashes.SHA256(),
					label=None,
				),
			)

			# Read and compress file
			with open(file_path, 'rb') as infile:
				data = infile.read()

			original_size = len(data)
			self.stats['total_bytes_processed'] += original_size

			cctx = zstd.ZstdCompressor(level=3)
			data = cctx.compress(data)
			self.ros_node.get_logger().info(
				f"File compressed: {len(data)} bytes (was {original_size} bytes)"
			)

			# AES-CBC encrypt with PKCS#7 padding
			cipher = Cipher(algorithms.AES(aes_key), modes.CBC(iv))
			encryptor = cipher.encryptor()
			pad_length = 16 - (len(data) % 16)
			padded_data = data + bytes([pad_length] * pad_length)
			encrypted_data = encryptor.update(padded_data) + encryptor.finalize()

			# Output file alongside input
			output_file = self.watch_dir / f"{file_path.name}.enc"

			metadata = {
				'original_name': file_path.name,
				'compression_method': 'zstd',
				'encryption_method': 'RSA-OAEP + AES-256-CBC',
				'key_length': len(encrypted_aes_key),
				'iv_length': len(iv),
				'timestamp': time.time(),
				'original_size': original_size,
				'compressed_size': len(data),
			}
			metadata_json = json.dumps(metadata).encode('utf-8')
			metadata_length = len(metadata_json).to_bytes(4, 'big')

			with open(output_file, 'wb') as outfile:
				outfile.write(metadata_length)
				outfile.write(metadata_json)
				outfile.write(encrypted_aes_key)
				outfile.write(iv)
				outfile.write(encrypted_data)

			self.ros_node.get_logger().info(f"Encrypted file saved: {output_file}")

			# Optionally delete the original file
			if self._secure_delete(file_path):
				self.ros_node.get_logger().info(f"Original file deleted: {file_path}")

			self.stats['files_encrypted'] += 1

		except Exception as e:
			self.ros_node.get_logger().error(f"Error while encrypting {file_path}: {e}")
			self.stats['files_failed'] += 1
		finally:
			self.processing_files.discard(file_path)

	# Watchdog hooks
	def on_created(self, event):
		if self.shutdown_requested:
			return
		if not event.is_directory and self._is_bag_file(event.src_path):
			self.ros_node.get_logger().info(f"New bag file detected: {event.src_path}")
			if self._is_file_complete(event.src_path):
				if self.shutdown_requested:
					return
				self._encrypt_file(event.src_path)

	def on_modified(self, event):
		if self.shutdown_requested:
			return
		if not event.is_directory and self._is_bag_file(event.src_path):
			if self._is_file_complete(event.src_path):
				if self.shutdown_requested:
					return
				self._encrypt_file(event.src_path)

	# Final pass on shutdown
	def process_final_files(self):
		self.ros_node.get_logger().info(
			f"Processing final files"
		)

		unencrypted_bags = self._find_unencrypted_bags()
		if not unencrypted_bags:
			self.ros_node.get_logger().info("No unencrypted bag files found")
			return

		self.ros_node.get_logger().info(
			f"Found {len(unencrypted_bags)} unencrypted bag file(s)"
		)
		for bag_file in unencrypted_bags:
			if self._wait_for_file_stability(bag_file, 2):
				self._encrypt_file(str(bag_file))
			else:
				self.ros_node.get_logger().warning(f"Skipping unstable file: {bag_file}")


class BagCryptoNode(Node):
	"""Minimal ROS2 node for bag file encryption"""

	def __init__(self):
		super().__init__('bag_crypto_node')

		# Parameters
		self.declare_parameter('watch_dir', '')
		self.declare_parameter('public_key_path', '')

		self.watch_dir = self.get_parameter('watch_dir').get_parameter_value().string_value
		self.public_key_path = self.get_parameter('public_key_path').get_parameter_value().string_value

		# Fallback to env var for public key
		if not self.public_key_path:
			self.public_key_path = os.getenv('PUBLIC_KEY_PATH', '')
			if not self.public_key_path:
				self.get_logger().error(
					'Error: missing public_key_path parameter and env var PUBLIC_KEY_PATH not set.'
				)
				raise ValueError('Missing public key path')

		if not self.watch_dir:
			self.get_logger().error('Error: watch_dir parameter is required')
			raise ValueError('Missing watch directory')

		if not os.path.exists(self.public_key_path):
			self.get_logger().error(f'Error: Public key not found: {self.public_key_path}')
			raise FileNotFoundError(f'Public key not found: {self.public_key_path}')

		self.get_logger().info('Bag Crypto Node initialized')
		self.get_logger().info(f'Watch directory: {self.watch_dir}')
		self.get_logger().info(f'Public key: {self.public_key_path}')

		self.event_handler = None
		self.observer = None
		self.is_running = False

		# Auto-start
		self.start_monitoring()

	def start_monitoring(self) -> bool:
		if self.is_running:
			self.get_logger().warning('Service is already running')
			return False
		try:
			self.event_handler = ROSBagCryptoHandler(
				self.public_key_path, self.watch_dir, self
			)
			self.observer = Observer()
			self.observer.schedule(self.event_handler, self.watch_dir, recursive=True)
			self.observer.start()
			self.is_running = True
			self.get_logger().info('Bag encryption monitoring started')
			return True
		except Exception as e:
			self.get_logger().error(f'Failed to start monitoring: {e}')
			return False

	def stop_monitoring(self) -> bool:
		if not self.is_running:
			return False
		try:
			if self.event_handler:
				self.event_handler.request_shutdown()
			if self.observer:
				self.observer.stop()
				# self.observer.join(0.0)
			self.is_running = False
			self.get_logger().info('Bag encryption monitoring stopped')
			return True
		except Exception as e:
			self.get_logger().error(f'Error stopping monitoring: {e}')
			return False

	def destroy_node(self):
		self.get_logger().info('Shutting down bag crypto node…')
		if self.is_running:
			self.stop_monitoring()
			if self.event_handler:
				self.event_handler.process_final_files()
		super().destroy_node()


def main(args=None):
	rclpy.init(args=args)

	node = BagCryptoNode()

	try:
		rclpy.spin(node)
	except (KeyboardInterrupt, ExternalShutdownException):
		node.get_logger().info(f"Received signal, shutting down gracefully…")
		node.destroy_node()
	except Exception as e:
		print(f"Error: {e}")
		if 'node' in locals():
			node.destroy_node()


if __name__ == '__main__':
	main()
