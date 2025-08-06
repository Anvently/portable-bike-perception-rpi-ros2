import os
import json
import math
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QUrl

class QuietHTTPRequestHandler(SimpleHTTPRequestHandler):
	def log_message(self, format, *args):
		pass

class MapWidget(QWidget):
	"""
	Widget pour afficher une carte interactive avec la trace GPS (Leaflet dynamique)
	"""

	def __init__(self, parent=None):
		super().__init__(parent)
		self.gps_data = []
		self.current_position = None
		self.server_root = os.path.join(os.path.dirname(__file__))  # cyclosafe_player/src
		self.temp_file = os.path.join(self.server_root, "gps_data.json")
		self.setup_ui()

	def start_server(self):
		os.chdir(self.server_root)  # Serve uniquement depuis ce dossier
		httpd = HTTPServer(('localhost', 8000), QuietHTTPRequestHandler)
		httpd.serve_forever()

	def setup_ui(self):
		layout = QVBoxLayout()
		self.title_label = QLabel("Trace GPS")
		self.title_label.setStyleSheet("font-weight: bold; margin: 5px;")
		layout.addWidget(self.title_label)

		self.web_view = QWebEngineView()
		layout.addWidget(self.web_view)
		self.setLayout(layout)

		# Lancer le serveur HTTP
		threading.Thread(target=self.start_server, daemon=True).start()

		# Copier le fichier HTML s'il n'existe pas
		html_template_src = os.path.join(self.server_root, "map_template.html")
		if not os.path.exists(html_template_src):
			raise FileNotFoundError("Fichier map_template.html introuvable dans cyclosafe_player/src")

		# Écriture initiale du JSON vide
		self.write_gps_json()

		# Afficher la carte
		self.display_map()

	def display_map(self):
		"""Charge la carte HTML dynamique dans le widget"""
		self.web_view.load(QUrl("http://localhost:8000/map_template.html"))

	def write_gps_json(self):
		"""Génère le fichier gps_data.json dans le dossier source"""
		trace = [[lat, lon] for lat, lon, _ in self.gps_data]
		current = list(self.current_position) if self.current_position else None

		data = {
			"trace": trace,
			"current": current
		}

		with open(self.temp_file, "w") as f:
			json.dump(data, f)

	def add_gps_point(self, latitude, longitude, rewrite=True, timestamp=None):
		if not math.isnan(latitude) and not math.isnan(longitude):
			self.gps_data.append((latitude, longitude, timestamp))
			if rewrite: self.write_gps_json()

	def set_current_position(self, latitude, longitude):
		if not math.isnan(latitude) and not math.isnan(longitude):
			self.current_position = (latitude, longitude)
			self.write_gps_json()

	def clear_data(self):
		self.gps_data = []
		self.current_position = None
		self.write_gps_json()
		self.title_label.setText("Trace GPS - Aucune donnée")

	def load_gps_data_from_bag(self, gps_messages):
		self.clear_data()

		for msg, timestamp in gps_messages:
			if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
				if not (math.isnan(msg.latitude) or math.isnan(msg.longitude)):
					self.add_gps_point(msg.latitude, msg.longitude, False, timestamp)
		self.write_gps_json()
		if self.gps_data:
			self.title_label.setText(f"Trace GPS ({len(self.gps_data)} points)")

	def update_current_position_from_message(self, msg):
		if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
			self.set_current_position(msg.latitude, msg.longitude)

	def load_rosbag(self, bag_path: str, topic_name: str):
		"""Charge un rosbag et initialise la carte GPS"""

		self.bag_path = bag_path

		gps_messages = []
		
		# Parcourir le rosbag pour extraire les messages GPS
		temp_reader = rosbag2_py.SequentialReader()
		storage_options = rosbag2_py._storage.StorageOptions(uri=self.bag_path, storage_id='mcap')
		converter_options = rosbag2_py._storage.ConverterOptions('', '')
		temp_reader.open(storage_options, converter_options)
		
		while temp_reader.has_next():
			(topic, data, t) = temp_reader.read_next()
			if topic == topic_name:
				msg_type = get_message("cyclosafe_interfaces/msg/NavSatInfo")
				msg = deserialize_message(data, msg_type)
				gps_messages.append((msg, t))
		# Charger les données dans la carte
		self.load_gps_data_from_bag(gps_messages)

	def closeEvent(self, event):
		if self.temp_file and os.path.exists(self.temp_file):
			try:
				os.unlink(self.temp_file)
			except:
				pass
		super().closeEvent(event)
