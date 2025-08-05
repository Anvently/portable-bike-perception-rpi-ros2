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
# https://cecill.info/licences/Licence_CeCILL_V2.1-en.html


import rclpy
from rclpy.executors import ExternalShutdownException
from picamera2 import Picamera2, Preview
from libcamera import Transform
from cyclosafe.src.ACamera import AImagePublisher
from typing import Any

class ImagePublisherPy(AImagePublisher):
	def init_camera(self):
		self.cam = Picamera2()
		capture_config = self.cam.create_video_configuration(
			main={
				"size": (self.resolution[0], self.resolution[1]),
				"format": "RGB888"
			},
			transform=Transform(vflip=False, hflip=False)
		)
		self.cam.configure(capture_config)
		if self.preview:
			self.cam.start_preview(Preview.DRM)
		self.count = 0
		#self.cam.options["quality"] = self.quality
		#self.cam.options["compress_level"] = 2
		self.cam.start()

	def capture(self) -> Any:
		"""Return image array"""
		image = self.cam.capture_array()
		# Vérification que l'image n'est pas vide
		if image is None or image.size == 0:
			self.get_logger().warn("Captured image is empty!")
			return
		
		# Vérification des dimensions de l'image
		self.get_logger().debug(f"Captured image shape: {image.shape}, dtype: {image.dtype}")
		
		return (image)

	def destroy(self):
		if hasattr(self, 'cam'):
			self.cam.stop()
		super().destroy_node()

def main(args=None):
	try:
		rclpy.init(args=args)
		image_pubisher = ImagePublisherPy()
		rclpy.spin(image_pubisher)
	
	except (KeyboardInterrupt, ExternalShutdownException):
		pass

	finally:
		if 'image_publisher' in locals():
			image_pubisher.cam.stop()
			image_pubisher.destroy_node()
		rclpy.shutdown()

if __name__ == '__main__':
	main()

