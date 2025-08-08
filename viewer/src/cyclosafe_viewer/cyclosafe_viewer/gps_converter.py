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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from cyclosafe_interfaces.msg import NavSatInfo
import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom
import math
from datetime import datetime, timezone

def nmea_to_decimal(nmea_coord):
	"""
	Convertit une coordonnée du format NMEA (DDMM.MMMMM) au format décimal.
	
	Args:
		nmea_coord (float): Coordonnée au format NMEA
		
	Returns:
		float: Coordonnée en format décimal
	"""

	nmea_str = str(nmea_coord)
	words = nmea_str.split('.')
	degres = 0
	if len(words[0]) <= 2:
		minutes = float(words[0] + '.' + words[1])
	else:
		degres = float(words[0][0:-2])
		minutes = float(words[0][-2:] + '.' + words[1])

	value = degres + (minutes / 60.0)

	return value

class GPSConverterNode(Node):

	def __init__(self):
		super().__init__("gps_converter")

		self.declare_parameter("topic_src", "/gps")
		self.declare_parameter("topic", "/nav_fix")
		self.declare_parameter("gpx_enabled", False)
		self.declare_parameter("gpx_file_path", "")
		self.declare_parameter("gpx_track_name", "ROS2 GPS Track")

		self.topic_src = self.get_parameter("topic_src").get_parameter_value().string_value
		self.topic = self.get_parameter("topic").get_parameter_value().string_value
		self.gpx_enabled = self.get_parameter("gpx_enabled").get_parameter_value().bool_value
		self.gpx_file_path = self.get_parameter("gpx_file_path").get_parameter_value().string_value
		self.gpx_track_name = self.get_parameter("gpx_track_name").get_parameter_value().string_value

		self.publisher = self.create_publisher(NavSatFix, self.topic, 10)
		self.suscriber = self.create_subscription(NavSatInfo, self.topic_src, self.gps_callback, 10)

		self.gpx_file = None
		self.gpx_root = None
		self.gpx_track = None
		self.gpx_segment = None
		if self.gpx_enabled:
			self.init_gpx_file()

		self.get_logger().info("GPS message converter initialized")
		if self.gpx_enabled:
			self.get_logger().info(f"GPX recording enabled, file: {self.gpx_file_path}")

	def gps_callback(self, msg: NavSatInfo):
		if msg.status.status == NavSatStatus.STATUS_FIX:
			fix = NavSatFix()
			fix.header.stamp = msg.header.stamp
			fix.status = msg.status
			fix.latitude = msg.latitude if not math.isnan(fix.latitude) else fix.latitude
			fix.longitude = msg.longitude if not math.isnan(fix.longitude) else fix.longitude
			self.publisher.publish(fix)

			if self.gpx_enabled:
				# Convertir le timestamp ROS en format ISO 8601 pour GPX
				ros_time = msg.header.stamp
				timestamp = datetime.fromtimestamp(ros_time.sec + ros_time.nanosec / 1e9).strftime("%Y-%m-%dT%H:%M:%SZ")
				
				self.add_gpx_point(
					lat=fix.latitude,
					lon=fix.longitude,
					alt=fix.altitude,
					timestamp=timestamp
				)

	def init_gpx_file(self):
		"""Initialise le fichier GPX avec les en-têtes et la structure nécessaire"""
		# Si aucun chemin n'est spécifié, on crée un fichier dans le répertoire courant
		if not self.gpx_file_path:
			timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
			self.gpx_file_path = f"gps_track_{timestamp}.gpx"
		
		# Création de la structure GPX
		self.gpx_root = ET.Element("gpx")
		self.gpx_root.set("version", "1.1")
		self.gpx_root.set("creator", "ROS2 GPS Converter Node")
		self.gpx_root.set("xmlns", "http://www.topografix.com/GPX/1/1")
		self.gpx_root.set("xmlns:xsi", "http://www.w3.org/2001/XMLSchema-instance")
		self.gpx_root.set("xsi:schemaLocation", "http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd")
		
		# Créer un élément metadata avec la date
		metadata = ET.SubElement(self.gpx_root, "metadata")
		name = ET.SubElement(metadata, "name")
		name.text = self.gpx_track_name
		time_elem = ET.SubElement(metadata, "time")
		time_elem.text = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
		
		# Créer la trace (track)
		self.gpx_track = ET.SubElement(self.gpx_root, "trk")
		track_name = ET.SubElement(self.gpx_track, "name")
		track_name.text = self.gpx_track_name
		
		# Créer un segment de trace
		self.gpx_segment = ET.SubElement(self.gpx_track, "trkseg")
		
		# Écrire l'en-tête dans le fichier
		self.save_gpx_file()

	def add_gpx_point(self, lat, lon, alt=None, timestamp=None):
		"""
		Ajoute un point à la trace GPX
		Args:
			lat (float): Latitude en degrés décimaux
			lon (float): Longitude en degrés décimaux
			alt (float, optional): Altitude en mètres
			timestamp (datetime, optional): Horodatage du point
		"""
		if not self.gpx_enabled or self.gpx_segment is None:
			return
			
		# Créer un point de trace
		trkpt = ET.SubElement(self.gpx_segment, "trkpt")
		trkpt.set("lat", str(lat))
		trkpt.set("lon", str(lon))
		
		# Ajouter l'altitude si disponible
		if alt is not None and not math.isnan(alt):
			ele = ET.SubElement(trkpt, "ele")
			ele.text = str(alt)
		
		# Ajouter l'horodatage si disponible
		if timestamp is not None:
			time_elem = ET.SubElement(trkpt, "time")
			time_elem.text = timestamp
			
		# Sauvegarder le fichier après chaque point
		self.save_gpx_file()
	
	def save_gpx_file(self):
		"""Sauvegarde le fichier GPX de manière formatée"""
		if not self.gpx_enabled or self.gpx_root is None:
			return
			
		# Conversion de l'arbre XML en chaîne et formatage
		rough_string = ET.tostring(self.gpx_root, encoding='utf-8')
		reparsed = minidom.parseString(rough_string)
		pretty_xml = reparsed.toprettyxml(indent="  ")
		
		# Écriture dans le fichier
		with open(self.gpx_file_path, "w") as f:
			f.write(pretty_xml)

def main(args=None):
	try:
		rclpy.init()
		visualizer = GPSConverterNode()
		rclpy.spin(visualizer)
	except ExternalShutdownException:
		pass

if __name__ == '__main__':
	main()
	