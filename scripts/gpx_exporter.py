#!/usr/bin/env python3
# -*- coding: utf-8 -*-
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


import argparse
import os
import sys
from datetime import datetime
import logging
import math
import rosbag2_py
from cyclosafe_interfaces.msg import NavSatInfo
from rich.logging import RichHandler

FORMAT = "%(message)s"
logging.basicConfig(
	level="NOTSET", format=FORMAT, datefmt="[%X]", handlers=[RichHandler()]
)  # set level=20 or logging.INFO to turn off debug
logger = logging.getLogger("rich")

# Import pour traiter les messages serialisés
try:
	import rclpy
	from rclpy.serialization import deserialize_message
	from rosidl_runtime_py.utilities import get_message
except ImportError:
	logger.error("Les packages ROS2 Python (rclpy) ne sont pas installés.")
	logger.error("Assurez-vous que ROS2 est correctement configuré.")
	sys.exit(1)


def nmea_to_decimal(nmea_coord):
	""" Convertit une coordonnée du format NMEA (DDMM.MMMMM) au format décimal.
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


def create_gpx_header(name="ROS2 GPS Track"):
	"""Crée l'en-tête du fichier GPX"""
	current_date = datetime.now().strftime("%Y-%m-%dT%H:%M:%SZ")
	
	header = f"""<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" 
	creator="ROS2 GPS Extractor"
	xmlns="http://www.topografix.com/GPX/1/1"
	xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
	xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd">
	<metadata>
		<name>{name}</name>
		<time>{current_date}</time>
	</metadata>
	<trk>
		<name>{name}</name>
		<trkseg>
"""
	return header


def create_gpx_footer():
	"""Crée le pied de page du fichier GPX"""
	return """        </trkseg>
	</trk>
</gpx>
"""


def create_gpx_point(lat, lon, ele=0.0, time=None):
	"""Crée un point GPX à partir des coordonnées"""
	time_str = f"<time>{time}</time>" if time else ""
	return f"""            <trkpt lat="{lat}" lon="{lon}">
				<ele>{ele}</ele>
				{time_str}
			</trkpt>
"""


def extract_gps_data(bag_path, output_path = None, track_name=None):
	"""
	Extrait les données GPS d'un bag ROS2 et les sauvegarde dans un fichier GPX
	
	Args:
		bag_path (str): Chemin vers le fichier bag ROS2
		output_path (str): Chemin du fichier GPX de sortie
		track_name (str, optional): Nom de la trace GPS
	"""
	if track_name is None:
		track_name = f"ROS2 GPS Track - {os.path.basename(bag_path)}"
	
	try:
		# Initialiser le reader de bag
		reader = rosbag2_py.SequentialReader()
		storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
		converter_options = rosbag2_py.ConverterOptions(
			input_serialization_format='cdr',
			output_serialization_format='cdr'
		)
		
		reader.open(storage_options, converter_options)
		
		# Obtenir les métadonnées du bag
		topic_types = reader.get_all_topics_and_types()
		
		# Créer un dictionnaire pour mapper les topics à leurs types
		type_map = {topic_type.name: topic_type.type for topic_type in topic_types}
		
		# Vérifier si le topic GPS existe
		if '/gps' not in type_map:
			logger.error(f"Le topic '/gps' n'existe pas dans le bag. Topics disponibles: {[t.name for t in topic_types]}")
			return False
		
		if output_path == None:
			parent_folder = os.path.dirname(bag_path)
			output_path = os.path.join(parent_folder, "trace.gpx")

		with open(output_path, 'w') as gpx_file:
			# Écrire l'en-tête GPX
			gpx_file.write(create_gpx_header(track_name))
			
			# Lire les messages du bag
			points_count = 0
			while reader.has_next():
				topic_name, data, timestamp = reader.read_next()
				
				if topic_name == '/gps':
					# Obtenir le type de message
					msg_type = get_message(type_map[topic_name])
					
					# Désérialiser le message
					msg: NavSatInfo = deserialize_message(data, msg_type)
					
					# Convertir les coordonnées NMEA en décimal
					try:
						if msg.status == - 1 or math.isnan(msg.latitude) or math.isnan(msg.longitude):
							continue
						# Supposons que msg contient des champs latitude et longitude
						lat = nmea_to_decimal(msg.latitude)
						lon = nmea_to_decimal(msg.longitude)
						
						# Utiliser l'altitude si disponible, sinon 0
						ele = getattr(msg, 'altitude', 0.0)
						
						# Utiliser le timestamp du message s'il existe
						timestamp_str = None
						if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
							timestamp_seconds = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
							timestamp_dt = datetime.fromtimestamp(timestamp_seconds)
							timestamp_str = timestamp_dt.strftime("%Y-%m-%dT%H:%M:%SZ")
						
						# Écrire le point dans le fichier GPX
						gpx_file.write(create_gpx_point(lat, lon, ele, timestamp_str))
						points_count += 1
						
					except AttributeError as e:
						logger.warning(f"Format de message GPS inattendu: {e}")
						logger.warning(f"Structure du message: {dir(msg)}")
						continue
			
			# Écrire le pied de page GPX
			gpx_file.write(create_gpx_footer())
			
			logger.info(f"Traitement terminé. {points_count} points GPS extraits et enregistrés dans {output_path}")
			return True
			
	except Exception as e:
		logger.error(f"Lors du traitement du bag: {e}")
		return False


def main():
	"""Fonction principale"""
	parser = argparse.ArgumentParser(description='Extrait les données GPS d\'un bag ROS2 et les exporte au format GPX')
	parser.add_argument('-b', '--bag', required=True, help='Chemin vers le fichier bag ROS2 (.mcap)')
	parser.add_argument('-o', '--output-path', help='Chemin du fichier GPX de sortie')
	parser.add_argument('-n', '--name', help='Nom personnalisé pour la trace GPS')
	
	args = parser.parse_args()
	
	# Vérifier que le fichier bag existe
	if not os.path.isfile(args.bag):
		logger.error(f"Le fichier bag {args.bag} n'existe pas.")
		return 1
	
	# Vérifier que le fichier bag est au format .mcap
	if not args.bag.lower().endswith('.mcap'):
		logger.warning("Le fichier bag ne semble pas être au format .mcap. Le traitement pourrait échouer.")
	
	# Extraire les données GPS
	success = extract_gps_data(args.bag, args.output_path, args.name)
	
	return 0 if success else 1


if __name__ == '__main__':
	# Initialiser ROS2 pour accéder aux types de messages
	rclpy.init()
	try:
		sys.exit(main())
	finally:
		# Nettoyer ROS2
		rclpy.shutdown()