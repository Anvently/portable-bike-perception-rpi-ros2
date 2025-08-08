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

import rclpy
import rclpy.qos
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Range
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from functools import partial
from std_msgs.msg import ColorRGBA
import math
from geometry_msgs.msg import Point

class RangeCircleVisualizer(Node):

	def __init__(self):
		# Initialisation du nœud ROS
		super().__init__("cyclosafe_visualizer")

		self.declare_parameter("topic_list", ["range1", "range2", "range3", "range4"])
		self.declare_parameter("colors", [0x40FF0000, 0x4000FF000, 0x400000FF, 0x40FFFF00])
	
		self.topic_list = self.get_parameter("topic_list").get_parameter_value().string_array_value
		self.colors = []
		self.colors_int32 = self.get_parameter("colors").get_parameter_value().integer_array_value
		for color_int32 in self.colors_int32:
			self.colors.append(ColorRGBA(
				a=float(((color_int32 & 0xFF000000) >> 24)) / 255.0,
				r=float(((color_int32 & 0x00FF0000) >> 16)) / 255.0,
				g=float(((color_int32 & 0x0000FF00) >> 8)) / 255.0,
				b=float(((color_int32 & 0x000000FF) >> 0)) / 255.0,
			))
		if len(self.colors) != len(self.topic_list):
			raise Exception(f"nbr of topic ({len(self.topic_list)}) doesn't match  number of colors ({len(self.colors)})")

		self.suscribers = {}
		self.publisher = self.create_publisher(Marker, '/visualization_marker',10)
		
		# Création du publisher pour les markers RViz
		for topic in self.topic_list:
			self.suscribers[topic] = self.create_subscription(Range, topic, partial(self.range_callback, topic=topic), 10)
			self.get_logger().info(f"Suscribed to topic {topic}")

	def range_callback(self, msg: Range, topic: str):
		# print(msg, msg.range, type(msg.range))
		# Création d'un marker de type CYLINDER qui représentera notre cercle
		marker = Marker()
		marker.header.frame_id = topic
		marker.header.stamp = msg.header.stamp
		index = self.topic_list.index(topic)
		
		marker.id = index
		marker.type = Marker.POINTS
		marker.action = Marker.ADD if not math.isnan(msg.range) else Marker.DELETE

		if marker.action == Marker.ADD:
			# Taille des points
			marker.scale.x = 0.03
			marker.scale.y = 0.03
			
			marker.color = self.colors[index]
			num_points = 1000  # Nombre de points dans le cercle
			radius = msg.range
			
			for i in range(num_points):
				angle = 2.0 * 3.14159 * i / num_points
				
				point = Point()
				point.x = radius * math.cos(angle)
				point.y = radius * math.sin(angle)
				point.z = 0.0
				
				marker.points.append(point)
			
		# Publier le marker
		# self.get_logger(f"published")
		self.publisher.publish(marker)
		

def main(args=None):
	try:
		rclpy.init()
		visualizer = RangeCircleVisualizer()
		rclpy.spin(visualizer)
	except ExternalShutdownException:
		pass

if __name__ == '__main__':
	main()

