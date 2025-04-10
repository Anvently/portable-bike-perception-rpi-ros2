from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
							QListWidget, QListWidgetItem, QInputDialog, 
							QColorDialog, QLabel, QTabWidget, QTreeWidget,
							QTreeWidgetItem, QFileDialog, QMessageBox, QGroupBox, QFormLayout,
							QDoubleSpinBox, QTextEdit)
from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor
from pyqtgraph import PlotWidget, InfiniteLine, mkPen
import rosbag2_py
from rclpy.serialization import deserialize_message
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
import json
import os
from datetime import datetime
from typing import List, Tuple, Any, Dict
from enum import IntEnum
from cyclosafe_player.src.PeakDetection import detect_peaks, Peak

class SonarDatas:
	# Define default colors for sonar graphs
	DEFAULT_COLORS = [QColor(255,0,0), QColor(0,255,0), QColor(0,0,255), 
					QColor(255,255,0), QColor(0,255,255), QColor(255,0,255),
					QColor(255,255,255)]
	
	nbr_instance = 0

	def __init__(self, topic_name:str, color:QColor = None):
		self.topic_name = topic_name
		if color:
			self.color = color
		else:
			idx = SonarDatas.get_sonar_index(topic_name)
			if idx != None: self.color = SonarDatas.DEFAULT_COLORS[idx]
			else: self.color = SonarDatas.DEFAULT_COLORS[SonarDatas.nbr_instance % len(SonarDatas.DEFAULT_COLORS)]
		self.datas: List[Tuple[float, float]] = []
		SonarDatas.nbr_instance += 1

	def get_sonar_index(topic_name) -> int:
		try:
			i = [x.isdigit() for x in topic_name].index(True)
			idx = int(topic_name[i]) - 1
			return idx
		except:
			pass
		return None
	

class MarkerCategoryEnum(IntEnum):
	User = 0
	Peak = 0

class MarkerCategory:
	"""Class to represent a type of marker with its color and visibility state."""

	def __init__(self, parent_tree: QTreeWidget, plot_widget: PlotWidget, name, color=Qt.red, visible=True, type=MarkerCategoryEnum.User):
		parent_tree
		self.name = name
		self.color = color if isinstance(color, str) else color.name()
		self.visible = visible
		self.markers: List[Marker] = []
		self.type = type
		self.plot_widget = plot_widget
		self.widget = QTreeWidgetItem(parent_tree)
		self.widget.setText(0, self.name)
		self.widget.setText(1, self.color)
		self.widget.setText(3, str(0))
		self.widget.setFlags(self.widget.flags() | Qt.ItemIsUserCheckable)
		self.widget.setCheckState(2, Qt.Checked if self.visible else Qt.Unchecked)
		self.widget.category = self
	
	def append_item(self, stamp: float, description: str = "", detail = None, color = None):
		self.markers.append(Marker(self, stamp, description, detail, color))
		self.widget.setText(3, str(len(self.markers)))
	
	def remove_item(self, item):
		if item in self.markers:
			self.plot_widget.removeItem(item.line)
			self.widget.removeChild(item.widget)
			self.markers.remove(item)
			self.widget.setText(3, str(len(self.markers)))

	# def show(self):
		# for item in self.markers:
			# 

class Marker:
	def __init__(self, category: MarkerCategory, stamp: float, description = "",  detail = None, color: QColor = None, display_label = False):
		self.stamp = stamp
		self.description = description
		self.detail = None
		self.category = category
		self.color = category.color if not color else color
		self.widget = QTreeWidgetItem(self.category.widget)
		self.widget.setText(0, f"{self.stamp:.2f}s")
		self.widget.setText(1, description)
		self.widget.marker = self

		if display_label:
			self.line: InfiniteLine = InfiniteLine(
				pos=self.stamp, 
				angle=90, 
				movable=False, 
				pen=mkPen(color=self.color, width=1, style=Qt.DashLine),
				label=description,
				labelOpts={'position': 0.9, 'color': self.color, 'fill': (0, 0, 0, 30)})
		else:
			self.line: InfiniteLine = InfiniteLine(
				pos=self.stamp, 
				angle=90, 
				movable=False, 
				pen=mkPen(color=self.color, width=1, style=Qt.DashLine))
		self.category.plot_widget.addItem(self.line)

class SonarGraphWidget(QWidget):
	"""Widget for displaying sonar data from ROS bag files as graphs."""

	def __init__(self, node: Node, parent=None):
		super().__init__(parent)
		self.node = node
		self.bag_path = None
		self.bag_info = None
		self.curves = {}
		self.marker_categories = {}  # Dictionary of marker types by name
		self.marker_lines = {}  # List to track marker line objects on the plot
		self.sonar_datas: Dict[str, SonarDatas] = {}
		self.init_ui()
	
	def init_ui(self):
		"""Initialize the UI components."""
		main_layout = QVBoxLayout(self)
		
		# Graph configuration header
		config_header = QLabel("Graphique des sonars")
		config_header.setStyleSheet("font-weight: bold;")
		main_layout.addWidget(config_header)
		
		# Curve controls layout
		controls_layout = QHBoxLayout()
		
		# Graph display widget
		self.plot_widget = PlotWidget()
		self.plot_widget.setLabel('left', 'Distance', units='m')
		self.plot_widget.setLabel('bottom', 'Temps', units='s')
		self.plot_widget.setTitle('Données des sonars')
		
		# Enable Y grid by default
		self.plot_widget.showGrid(x=False, y=True)
		
		# Current time marker (vertical line)
		self.current_time_marker = InfiniteLine(pos=0, angle=90, movable=False, pen='w')
		self.plot_widget.addItem(self.current_time_marker)
		
		# Arrange the layouts
		controls_layout.addWidget(self.plot_widget, 3)  # Give the plot more space
		
		main_layout.addLayout(controls_layout)
	
	def init_tab_ui(self, parent_tab_widget: QTabWidget):
		"""Initialize the tabs for curve and marker controls."""
		# --- Tab 1: Curves ---
		curves_tab = QWidget()
		curves_layout = QVBoxLayout()
		curves_tab.setLayout(curves_layout)
		
		curve_list_label = QLabel("Courbes affichées:")
		curves_layout.addWidget(curve_list_label)
		
		self.curve_list = QListWidget()
		self.curve_list.itemChanged.connect(self.curve_visibility_changed)
		curves_layout.addWidget(self.curve_list)
		
		# Add/Remove buttons for curves
		curves_btn_layout = QHBoxLayout()
		self.add_curve_button = QPushButton("+")
		self.remove_curve_button = QPushButton("-")
		self.add_curve_button.clicked.connect(self.on_add_curve)
		self.remove_curve_button.clicked.connect(self.on_remove_curve)
		curves_btn_layout.addWidget(self.add_curve_button)
		curves_btn_layout.addWidget(self.remove_curve_button)
		curves_layout.addLayout(curves_btn_layout)
		
		parent_tab_widget.addTab(curves_tab, "Courbes")
		
		# --- Tab 2: Markers ---
		markers_tab = QWidget()
		markers_layout = QVBoxLayout()
		markers_tab.setLayout(markers_layout)
		
		# Marker types tree
		markers_layout.addWidget(QLabel("Types de marqueurs:"))
		self.marker_tree = QTreeWidget()
		self.marker_tree.setHeaderLabels(["Type", "Couleur", "Visible", "Nombre"])
		self.marker_tree.setColumnWidth(0, 150)
		self.marker_tree.itemChanged.connect(self.marker_tree_item_changed)
		self.marker_tree.itemSelectionChanged.connect(self.marker_tree_item_selected)
		markers_layout.addWidget(self.marker_tree)
		
		# Marker type controls
		type_controls = QHBoxLayout()
		self.add_type_button = QPushButton("Nouveau type")
		self.add_type_button.clicked.connect(self.on_add_marker_category)
		self.remove_type_button = QPushButton("Supprimer type")
		self.remove_type_button.clicked.connect(self.on_remove_marker_category)
		self.change_color_button = QPushButton("Changer couleur")
		self.change_color_button.clicked.connect(self.on_change_marker_color)
		type_controls.addWidget(self.add_type_button)
		type_controls.addWidget(self.remove_type_button)
		type_controls.addWidget(self.change_color_button)
		markers_layout.addLayout(type_controls)
		
		# Marker controls
		marker_controls = QHBoxLayout()
		self.add_marker_button = QPushButton("Ajouter marqueur")
		self.add_marker_button.clicked.connect(self.on_add_marker)
		self.remove_marker_button = QPushButton("Supprimer marqueur")
		self.remove_marker_button.clicked.connect(self.on_remove_marker)
		marker_controls.addWidget(self.add_marker_button)
		marker_controls.addWidget(self.remove_marker_button)
		markers_layout.addLayout(marker_controls)
		
		# Import/Export controls
		io_controls = QHBoxLayout()
		self.import_button = QPushButton("Importer")
		self.import_button.clicked.connect(self.on_import_markers)
		self.export_button = QPushButton("Exporter")
		self.export_button.clicked.connect(self.on_export_markers)
		io_controls.addWidget(self.import_button)
		io_controls.addWidget(self.export_button)
		markers_layout.addLayout(io_controls)

		markers_layout.addWidget(QLabel("Détails"))
		self.marker_details_text = QTextEdit()
		self.marker_details_text.setReadOnly(True)
		self.marker_details_text.setMinimumHeight(250)
		markers_layout.addWidget(self.marker_details_text)
		
		parent_tab_widget.addTab(markers_tab, "Marqueurs")

		# --- Tab 3: Outils ---
		tools_tab = QWidget()
		tools_layout = QVBoxLayout()
		tools_tab.setLayout(tools_layout)

		# Groupbox pour l'outil de détection de pics
		peak_detector_group = QGroupBox("Détection de pics (sonar)")
		peak_detector_layout = QVBoxLayout()
		peak_detector_group.setLayout(peak_detector_layout)

		# Paramètres de l'outil
		params_form = QFormLayout()

		# Seuil minimal
		self.threshold_spinbox = QDoubleSpinBox()
		self.threshold_spinbox.setRange(0.01, 5.0)
		self.threshold_spinbox.setValue(3.0)
		self.threshold_spinbox.setSingleStep(0.05)
		self.threshold_spinbox.setDecimals(2)
		self.threshold_spinbox.setSuffix(" m")
		self.threshold_spinbox.setToolTip("Seuil minimal de détection d'un pic par rapport à sa valeur seuil")
		params_form.addRow("Seuil minimal:", self.threshold_spinbox)

		# Durée minimale
		self.min_duration_spinbox = QDoubleSpinBox()
		self.min_duration_spinbox.setRange(0.00, 10.0)
		self.min_duration_spinbox.setValue(0.3)
		self.min_duration_spinbox.setSingleStep(0.1)
		self.min_duration_spinbox.setDecimals(2)
		self.min_duration_spinbox.setSuffix(" s")
		self.min_duration_spinbox.setToolTip("Durée minimale pour qu'un pic soit considéré comme valide")
		params_form.addRow("Durée minimale:", self.min_duration_spinbox)

		# Tolérance de rétablissement
		self.recovery_tolerance_spinbox = QDoubleSpinBox()
		self.recovery_tolerance_spinbox.setRange(0.00, 5.0)
		self.recovery_tolerance_spinbox.setValue(0.3)
		self.recovery_tolerance_spinbox.setSingleStep(0.1)
		self.recovery_tolerance_spinbox.setDecimals(2)
		self.recovery_tolerance_spinbox.setSuffix(" s")
		self.recovery_tolerance_spinbox.setToolTip("Durée maximale pour laquelle un rétablissement bref d'un pic est ignoré")
		params_form.addRow("Tolérance rétablissement:", self.recovery_tolerance_spinbox)

		peak_detector_layout.addLayout(params_form)

		# Liste des sonars disponibles
		self.sonar_list_label = QLabel("Sonars à analyser:")
		peak_detector_layout.addWidget(self.sonar_list_label)

		self.sonar_list = QListWidget()
		self.sonar_list.setSelectionMode(QListWidget.SelectionMode.MultiSelection)
		peak_detector_layout.addWidget(self.sonar_list)

		# Bouton pour lancer l'analyse
		self.detect_peaks_button = QPushButton("Lancer la détection de pics")
		self.detect_peaks_button.clicked.connect(self.on_detect_peaks)
		peak_detector_layout.addWidget(self.detect_peaks_button)

		# Cadre pour afficher les résultats d'analyse
		results_group = QGroupBox("Résultats de l'analyse")
		results_layout = QVBoxLayout()
		results_group.setLayout(results_layout)

		# Zone de texte pour afficher les résultats
		self.results_text_edit = QTextEdit()
		self.results_text_edit.setReadOnly(True)  # En lecture seule
		self.results_text_edit.setMinimumHeight(150)  # Hauteur minimale
		results_layout.addWidget(self.results_text_edit)

		# Ajouter les groupes à l'onglet
		tools_layout.addWidget(peak_detector_group)
		tools_layout.addWidget(results_group)

		# Espace vertical pour d'éventuels futurs outils
		tools_layout.addStretch()

		parent_tab_widget.addTab(tools_tab, "Outils")
	
	def set_bag_info(self, bag_path, bag_info):
		"""Set the bag file information and path."""
		self.bag_path = bag_path
		self.bag_info = bag_info
		self.clear_all_curves()
		
		self.load_sonar_data()

		# Auto-add sonar topics with default colors
		for sonar in self.sonar_datas:
			self.add_curve(sonar, False)

		self.populate_sonar_list()
	
	def clear_all_curves(self):
		"""Clear all curves from the plot."""
		self.plot_widget.clear()
		self.curves = {}
		self.curve_list.clear()
		# Re-add the time marker after clearing
		self.current_time_marker = InfiniteLine(pos=0, angle=90, movable=False, pen='w')
		self.plot_widget.addItem(self.current_time_marker)
		# Re-draw any visible markers
		self.update_marker_display()
	
	def on_add_curve(self, topic=None, color=None):
		"""Add a curve to the plot."""
		if not self.bag_info:
			return
		
		# Filter to only Range topics
		topics = list(self.sonar_datas.keys())
		
		if not topics:
			self.node.get_logger().warning("No Range topics available")
			return
		
		# Handle topic selection
		if topic is None or topic is False:
			topic, ok = QInputDialog.getItem(self, "Ajouter une courbe", 
										"Sélectionner un topic Range:", 
										topics, 0, False)
		elif topic not in topics:
			self.node.get_logger().warning(f"Could not load topic {topic} on graph")
			ok = False
		else:
			ok = True
			
		if not ok or not topic:
			return
			
		# # Handle color selection
		# if color is None:
		# 	color = QColorDialog.getColor()
		# 	if not color.isValid():
		# 		color = Qt.red
		
		# # Use the chosen or default color
		# color_name = color if isinstance(color, str) else color.name()
		
		self.add_curve(topic)
	
	def add_curve(self, topic, visible=True):
		"""Add a curve with the given topic and color."""
		# Load data for the topic
		if not topic in self.sonar_datas:
			self.node.get_logger().warning(f"No data for {topic}")
			return
		
		sonar_data = self.sonar_datas[topic]

		x_vals, y_vals = self.process_messages(sonar_data.datas)
		
		# Plot the curve with the chosen color
		curve_item = self.plot_widget.plot(x_vals, y_vals, pen=sonar_data.color, name=topic)
		if not visible:
			curve_item.setVisible(False)

		# Keep reference to the curve
		self.curves[topic] = curve_item
		
		# Add to the list of displayed curves
		item = QListWidgetItem(topic)
		item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
		item.setCheckState(Qt.Checked if visible else Qt.Unchecked)
		self.curve_list.addItem(item)
		
		return curve_item
	
	def on_remove_curve(self):
		"""Remove selected curves from the plot."""
		selected_items = self.curve_list.selectedItems()
		if not selected_items:
			return
			
		for item in selected_items:
			topic = item.text()
			if topic in self.curves:
				self.plot_widget.removeItem(self.curves[topic])
				del self.curves[topic]
				
			row = self.curve_list.row(item)
			self.curve_list.takeItem(row)
	
	def curve_visibility_changed(self, item):
		"""Toggle curve visibility when checkbox state changes."""
		topic = item.text()
		if topic in self.curves:
			curve_item = self.curves[topic]
			curve_item.setVisible(item.checkState() == Qt.Checked)
	
	def update_current_time(self, time_seconds):
		"""Update the position of the current time marker."""
		self.current_time_marker.setValue(time_seconds)
	
	def load_sonar_data(self):
		"""
		Read all messages of sonar type and save them in a list of
		of tuples (rel_time, range_value),
		where rel_time is the time (in seconds) relative to the bag's start_time.
		"""
		if not self.bag_info or not self.bag_path:
			return

		for topic_str, topic in self.bag_info.topic_info.items():
			if topic.msg_type == "sensor_msgs/msg/Range":
				self.sonar_datas[topic_str] = SonarDatas(topic_str)
		
		if (len(self.sonar_datas) == 0):
			return
		
		msg_class = get_message("sensor_msgs/msg/Range")
		
		# Open the bag with rosbag2_py
		reader = rosbag2_py.SequentialReader()
		storage_options = rosbag2_py._storage.StorageOptions(uri=self.bag_path, storage_id='mcap')
		converter_options = rosbag2_py._storage.ConverterOptions('', '')
		reader.open(storage_options, converter_options)
		
		self.data = []
		start_time = self.bag_info.start_time  # in ns
		
		# Read all messages
		while reader.has_next():
			(tpc, raw_data, t) = reader.read_next()
			if tpc in self.sonar_datas:
				msg = deserialize_message(raw_data, msg_class)
				# Calculate relative time in seconds
				rel_time = (t - start_time) / 1e9
				# Add range value
				self.sonar_datas[tpc].datas.append((rel_time, msg.range))
				

	# def load_topic_data(self, topic):
	# 	"""
	# 	Read all messages from the given topic (only if Range type)
	# 	and return a list of tuples (rel_time, range_value),
	# 	where rel_time is the time (in seconds) relative to the bag's start_time.
	# 	"""
	# 	if not self.bag_info or not self.bag_path:
	# 		return []
			
	# 	# Verify topic type
	# 	topic_type_str = self.bag_info.topic_info[topic].msg_type
	# 	if topic_type_str != "sensor_msgs/msg/Range":
	# 		self.node.get_logger().warning(f"Topic {topic} is not of type Range")
	# 		return []
		
	# 	msg_class = get_message(topic_type_str)
		
	# 	# Open the bag with rosbag2_py
	# 	reader = rosbag2_py.SequentialReader()
	# 	storage_options = rosbag2_py._storage.StorageOptions(uri=self.bag_path, storage_id='mcap')
	# 	converter_options = rosbag2_py._storage.ConverterOptions('', '')
	# 	reader.open(storage_options, converter_options)
		
	# 	data = []
	# 	start_time = self.bag_info.start_time  # in ns
		
	# 	# Read all messages
	# 	while reader.has_next():
	# 		(tpc, raw_data, t) = reader.read_next()
	# 		if tpc == topic:
	# 			msg = deserialize_message(raw_data, msg_class)
	# 			# Calculate relative time in seconds
	# 			rel_time = (t - start_time) / 1e9
	# 			# Add range value
	# 			data.append((rel_time, msg.range))
				
	# 	return data
	
	def process_messages(self, data):
		"""
		From a list of tuples (rel_time, range_value),
		return two lists: x_vals (time in seconds) and y_vals (range value).
		"""
		x_vals = [t for t, r in data]
		y_vals = [r for t, r in data]
		return x_vals, y_vals
	
	# --- Marker Management ---
	
	def on_add_marker_category(self):
		"""Add a new marker type."""
		type_name, ok = QInputDialog.getText(self, "Nouveau type de marqueur", 
										"Nom du type de marqueur:")
		if ok and type_name:
			if type_name in self.marker_categories:
				QMessageBox.warning(self, "Type existant", 
								f"Le type de marqueur '{type_name}' existe déjà.")
				return
				
			color = QColorDialog.getColor()
			if not color.isValid():
				color = Qt.red
				
			# Create new marker type
			self.marker_categories[type_name] = MarkerCategory(self.marker_tree, self.plot_widget, type_name, color, visible=True)

	
	def on_remove_marker_category(self):
		"""Remove the selected marker type."""
		selected_items = self.marker_tree.selectedItems()
		if not selected_items:
			return
			
		for item in selected_items:
			# Only process top-level items (marker types)
			if item.parent() is None:
				type_name = item.text(0)
				if type_name in self.marker_categories:
					self.
					del self.marker_categories[type_name]
		
		# Update tree and display
		self.update_marker_tree()
		self.update_marker_display()
	
	def on_change_marker_color(self):
		"""Change the color of the selected marker type."""
		selected_items = self.marker_tree.selectedItems()
		if not selected_items:
			return
			
		for item in selected_items:
			# Only process top-level items (marker types)
			if item.parent() is None:
				type_name = item.text(0)
				if type_name in self.marker_categories:
					color = QColorDialog.getColor()
					if color.isValid():
						self.marker_categories[type_name].color = color.name()
						# Update tree display
						item.setText(1, color.name())
						# Update markers on plot
						self.update_marker_display()
	
	def on_add_marker(self):
		"""Add a new marker at the current time or a specified time."""
		selected_items = self.marker_tree.selectedItems()
		type_name = None
		
		# Get selected marker type
		for item in selected_items:
			if item.parent() is None:  # It's a type item
				type_name = item.text(0)
				break
		
		if not type_name:
			types = list(self.marker_categories.keys())
			if not types:
				QMessageBox.warning(self, "Aucun type", 
								"Veuillez créer un type de marqueur d'abord.")
				return
				
			type_name, ok = QInputDialog.getItem(self, "Ajouter un marqueur", 
											"Type de marqueur:", 
											types, 0, False)
			if not ok or not type_name:
				return
		
		# Get timestamp (default to current marker position)
		current_time = self.current_time_marker.value()
		timestamp, ok = QInputDialog.getDouble(self, "Timestamp", 
											"Temps (secondes):", 
											current_time, 0, 
											self.bag_info.duration if self.bag_info else 999999, 2)
		if not ok:
			return
			
		# Get description
		description, ok = QInputDialog.getText(self, "Description", 
											"Description du marqueur:")
		if not ok:
			description = f"Marqueur à {timestamp:.2f}s"
			
		# Add marker
		self.marker_categories[type_name].markers.append((timestamp, description, None))
		
		# Update tree and display
		self.update_marker_tree()
		self.update_marker_display()
	
	def on_remove_marker(self):
		"""Remove the selected marker."""
		selected_items = self.marker_tree.selectedItems()
		if not selected_items:
			return
			
		for item in selected_items:
			# Only process child items (markers)
			if item.parent() is not None:
				type_name = item.parent().text(0)
				if type_name in self.marker_categories:
					marker_index = item.parent().indexOfChild(item)
					if 0 <= marker_index < len(self.marker_categories[type_name].markers):
						del self.marker_categories[type_name].markers[marker_index]
		
		# Update tree and display
		self.update_marker_tree()
		self.update_marker_display()
	
	def marker_tree_item_changed(self, item, column):
		"""Handle changes in the marker tree."""
		if column == 2 and item.parent() is None:  # Visibility column for type
			type_name = item.text(0)
			if type_name in self.marker_categories:
				self.marker_categories[type_name].visible = (item.checkState(2) == Qt.Checked)
				self.update_marker_display()
	
	def marker_tree_item_selected(self):
		selected_item = self.marker_tree.selectedItems()
		if selected_item:
			item = selected_item[0]
			if item.parent() == None:
				return
			text = f"stamp={item.timestamp}\ndescription={item.description}\n"
			if item.detail != None and hasattr(item.detail, "__str__"):
				text += str(item.detail)
			self.marker_details_text.setText(text)

	def update_marker_tree(self):
		"""Update the marker tree display."""
		self.marker_tree.clear()
		
		for type_name, marker_category in self.marker_categories.items():
			type_item = QTreeWidgetItem(self.marker_tree)
			type_item.setText(0, type_name)
			type_item.setText(1, marker_category.color)
			type_item.setText(3, str(len(marker_category.markers)))
			type_item.setFlags(type_item.flags() | Qt.ItemIsUserCheckable)
			type_item.setCheckState(2, Qt.Checked if marker_category.visible else Qt.Unchecked)
			# Add marker children
			self.results_text_edit.setText(f"Adding markers for {type_name}")
			for timestamp, description, detail in marker_category.markers:
				marker_item = QTreeWidgetItem(type_item)
				marker_item.setText(0, f"{timestamp:.2f}s")
				marker_item.setText(1, description)
				marker_item.timestamp = timestamp
				marker_item.description = description
				marker_item.detail = detail
		self.marker_tree.itemWidget()
			
	
	def update_marker_display(self):
		"""Update the display of markers on the plot."""
		# Remove all existing marker lines
		for type in self.marker_lines:
			for line in self.marker_lines[type]:
				self.plot_widget.removeItem(line)
		self.marker_lines = {}
		
		# Add lines for visible marker types
		for type_name, marker_category in self.marker_categories.items():
			self.marker_lines[type_name] = []
			if marker_category.visible:
				for timestamp, description, _ in marker_category.markers:
					# Create a vertical line at the marker position
					line = InfiniteLine(
						pos=timestamp, 
						angle=90, 
						movable=False, 
						pen=mkPen(color=marker_category.color, width=1, style=Qt.DashLine),
						label=description,
						labelOpts={'position': 0.9, 'color': marker_category.color, 'fill': (0, 0, 0, 30)}
					)
					self.plot_widget.addItem(line)
					self.marker_lines[type_name].append(line)
			self.results_text_edit.setText(f"Adding marker lines for {type_name}")
	
	def on_import_markers(self):
		"""Import markers from a JSON file."""
		file_path, _ = QFileDialog.getOpenFileName(
			self, "Importer des marqueurs", "", "Fichiers JSON (*.json)"
		)
		
		if not file_path:
			return
			
		try:
			with open(file_path, 'r') as f:
				data = json.load(f)
				
			# Clear existing markers
			self.marker_categories = {}
			
			# Import marker types and markers
			for type_data in data.get('marker_categories', []):
				type_name = type_data.get('name')
				if type_name:
					marker_category = MarkerCategory(
						type_name,
						type_data.get('color', '#FF0000'),
						type_data.get('visible', True)
					)
					
					# Import markers for this type
					for marker_data in type_data.get('markers', []):
						timestamp = marker_data.get('timestamp', 0.0)
						description = marker_data.get('description', '')
						detail = marker_data.get('detail', None)
						marker_category.markers.append((timestamp, description, detail))
					
					self.marker_categories[type_name] = marker_category
			
			# Update display
			self.update_marker_tree()
			self.update_marker_display()
			
			QMessageBox.information(self, "Import réussi", 
								f"Marqueurs importés avec succès depuis {file_path}")
								
		except Exception as e:
			QMessageBox.critical(self, "Erreur d'import", 
							f"Erreur lors de l'import des marqueurs: {str(e)}")
	
	def on_export_markers(self):
		"""Export markers to a JSON file."""
		if not self.marker_categories:
			QMessageBox.information(self, "Aucun marqueur", 
								"Aucun marqueur à exporter.")
			return
		
		# Generate default filename with timestamp
		default_filename = f"markers_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
		
		file_path, _ = QFileDialog.getSaveFileName(
			self, "Exporter les marqueurs", default_filename, "Fichiers JSON (*.json)"
		)
		
		if not file_path:
			return
			
		try:
			# Prepare data structure
			data = {
				'marker_categories': []
			}
			
			for type_name, marker_category in self.marker_categories.items():
				type_data = {
					'name': type_name,
					'color': marker_category.color,
					'visible': marker_category.visible,
					'markers': []
				}
				
				for timestamp, description, detail in marker_category.markers:
					type_data['markers'].append({
						'timestamp': timestamp,
						'description': description,
						'detail': detail
					})
				
				data['marker_categories'].append(type_data)
			
			# Write to file
			with open(file_path, 'w') as f:
				json.dump(data, f, indent=2)
				
			QMessageBox.information(self, "Export réussi", 
								f"Marqueurs exportés avec succès vers {file_path}")
								
		except Exception as e:
			QMessageBox.critical(self, "Erreur d'export", 
							f"Erreur lors de l'export des marqueurs: {str(e)}")
			
	def populate_sonar_list(self):
		"""Populate the sonar list with available Range message topics."""
		self.sonar_list.clear()
		
		# Parcourir tous les topics
		for topic in self.sonar_datas:
			# Créer un item pour ce sonar
			item = QListWidgetItem(topic)
			item.setData(Qt.ItemDataRole.UserRole, topic)
			item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
			item.setCheckState(Qt.CheckState.Unchecked)
			self.sonar_list.addItem(item)

	def on_detect_peaks(self):
		"""Launch peak detection for selected sonars."""
		# Récupérer les paramètres
		threshold = self.threshold_spinbox.value()
		min_duration = self.min_duration_spinbox.value()
		recovery_tolerance = self.recovery_tolerance_spinbox.value()
		
		# Récupérer les sonars sélectionnés
		selected_sonars = []
		for i in range(self.sonar_list.count()):
			item = self.sonar_list.item(i)
			if item.checkState() == Qt.CheckState.Checked:
				topic = item.data(Qt.ItemDataRole.UserRole)
				selected_sonars.append(topic)
		
		if not selected_sonars:
			QMessageBox.warning(self, "Aucun sonar sélectionné", 
							"Veuillez sélectionner au moins un sonar pour l'analyse.")
			return
		
		results = {}
		for sonar_topic in selected_sonars:
			# Récupérer les données du sonar
			sonar_data = self.sonar_datas[sonar_topic].datas
			
			# Lancer la détection
			self.results_text_edit.setText(f"Starting peak detection for {sonar_topic}")
			peaks = detect_peaks(sonar_data, 
								threshold_drop=threshold,
								min_duration_sec=min_duration, 
								recovery_tolerance_sec=recovery_tolerance,
								feedback_handler=self.results_text_edit.setText)
			results[sonar_topic] = peaks
		
		self.handle_peak_detection_results(results)

	def handle_peak_detection_results(self, results: Dict[str, List[Peak]]):
		if not results or len(results) == 0:
			return
		results_text = ""
		for topic, peaks in results.items():
			marker_category = f"{topic}_peaks"
			if not marker_category in self.marker_categories:
				self.marker_categories[marker_category] = MarkerCategory(marker_category, self.sonar_datas[topic].color, True, MarkerCategoryEnum.Peak)
			else:
				self.marker_categories[marker_category].markers = []
			for peak in peaks:
				self.marker_categories[marker_category].markers.append((peak.start_time, "{:.2f}".format(peak.mean), peak))
			results_text += f"{topic}: {len(peaks)} pics détectés\n"

		self.update_marker_tree()
		self.update_marker_display()
		self.results_text_edit.setText(results_text)

