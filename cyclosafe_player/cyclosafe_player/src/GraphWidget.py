from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
							QListWidget, QListWidgetItem, QInputDialog, 
							QColorDialog, QLabel, QTabWidget, QTreeWidget,
							QScrollArea, QFileDialog, QMessageBox, QGroupBox, QFormLayout,
							QDoubleSpinBox, QTextEdit, QCheckBox)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QBrush
from pyqtgraph import PlotWidget, InfiniteLine
import rosbag2_py
from rclpy.serialization import deserialize_message
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
import json, os
from datetime import datetime
from typing import List, Tuple, Any, Dict
from cyclosafe_player.src.PeakDetection import detect_peaks, Peak
from cyclosafe_player.src.Marker import Marker, MarkerCategory, MarkerCategoryEnum, GenericMarkerDialog, OvertakeMarkerDialog

def find_peak_in_category(category: MarkerCategory, time: float, tolerance = 0.0) -> Marker:
	if category.type != MarkerCategoryEnum.Peak:
		raise Exception("Invalid category type")
	for _, marker in category.markers.items():
		peak: Peak = marker.detail
		if peak.start_time - tolerance <= time <= peak.start_time + peak.duration + tolerance:
			return marker
	return None

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
		self.highlight: bool = True
		SonarDatas.nbr_instance += 1

	def get_sonar_index(topic_name) -> int:
		try:
			i = [x.isdigit() for x in topic_name].index(True)
			idx = int(topic_name[i]) - 1
			return idx
		except:
			pass
		return None

class SonarGraphWidget(QWidget):
	"""Widget for displaying sonar data from ROS bag files as graphs."""

	def __init__(self, node: Node, parent=None):
		super().__init__(parent)
		self.node = node
		self.bag_path = None
		self.bag_info = None
		self.curves = {}
		self.marker_categories: Dict[str, MarkerCategory] = {}  # Dictionary of marker types by name
		self.marker_lines = {}  # List to track marker line objects on the plot
		self.sonar_datas: Dict[str, SonarDatas] = {}
		self.peaks: Dict[str, List[Peak]] = {}
		self.previous_selected_marker = None
		self.current_peaks: Dict[str, Marker] = {}
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

		Marker.link_plot(self.plot_widget)
	
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
		markers_layout.addWidget(QLabel("Catégories de marqueurs:"))
		self.marker_tree = QTreeWidget()
		self.marker_tree.setHeaderLabels(["Type", "Couleur", "Visible", "Nombre"])
		self.marker_tree.setColumnWidth(0, 150)
		self.marker_tree.itemChanged.connect(self.marker_tree_item_changed)
		self.marker_tree.itemSelectionChanged.connect(self.marker_tree_item_selected)
		markers_layout.addWidget(self.marker_tree)
		
		# Marker type controls
		type_controls = QHBoxLayout()
		self.add_type_button = QPushButton("Nouvelle catégorie")
		self.add_type_button.clicked.connect(self.on_add_marker_category)
		self.remove_type_button = QPushButton("Supprimer catégorie")
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

		MarkerCategory.link_tree(self.marker_tree)
		MarkerCategory.dft_new_marker_handler = lambda: GenericMarkerDialog(self.current_time_marker.value(), self.bag_info.duration, self)()

		# --- Tab 3: Outils ---
		tools_tab = QWidget()
		tools_layout = QVBoxLayout()
		tools_tab.setLayout(tools_layout)

		# Groupbox pour l'outil de détection de pics
		peak_detector_group = QGroupBox("Détection de pics")
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

		csv_tool_group = QGroupBox("Importer/exporter en csv")
		csv_tool_layout = QVBoxLayout()
		csv_tool_group.setLayout(csv_tool_layout)

		# Paramètres de l'outil
		csv_params_form = QFormLayout()

		self.csv_import_button = QPushButton("Importer csv")
		self.csv_import_button.clicked.connect(self.on_import_csv)
		csv_tool_layout.addWidget(self.csv_import_button)
		self.csv_export_button = QPushButton("Exporter csv")
		self.csv_export_button.clicked.connect(self.on_export_csv)
		csv_tool_layout.addWidget(self.csv_export_button)

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
		tools_layout.addWidget(csv_tool_group)
		tools_layout.addWidget(results_group)

		# Espace vertical pour d'éventuels futurs outils
		tools_layout.addStretch()

		parent_tab_widget.addTab(tools_tab, "Outils")

		 # --- Tab 4: Analyse ---
		analysis_tab = QWidget()
		analysis_layout = QVBoxLayout()
		analysis_tab.setLayout(analysis_layout)

		# Zone de défilement principale
		scroll_area = QScrollArea()
		scroll_area.setWidgetResizable(True)
		scroll_content = QWidget()
		self.analysis_content_layout = QVBoxLayout(scroll_content)
		scroll_area.setWidget(scroll_content)
		analysis_layout.addWidget(scroll_area)

		# Dictionnaire pour stocker les widgets liés à chaque sonar
		self.sonar_analysis_widgets = {}

		# Ajouter un espace extensible à la fin
		self.analysis_content_layout.addStretch()

		parent_tab_widget.addTab(analysis_tab, "Analyse")

	def update_sonar_analysis_tab(self):
		"""Met à jour l'onglet d'analyse avec les sonars actuels."""
		# Supprimer les widgets existants
		for widgets in self.sonar_analysis_widgets.values():
			widgets['group'].deleteLater()
		
		self.sonar_analysis_widgets.clear()
		
		# Créer des sections pour chaque sonar dans le dictionnaire
		for sonar_name in self.sonar_datas.keys():
			# Créer un groupe pour ce sonar
			group = QWidget()
			group_layout = QVBoxLayout(group)
			group_layout.setContentsMargins(5, 5, 5, 5)
			
			# En-tête avec checkbox et bouton de dévoilement
			header = QWidget()
			header_layout = QHBoxLayout(header)
			header_layout.setContentsMargins(0, 0, 0, 0)
			
			# Checkbox pour activer/désactiver la mise en évidence
			highlight_checkbox = QCheckBox("Mettre en évidence")
			highlight_checkbox.setObjectName(f"highlight_{sonar_name}")
			highlight_checkbox.setCheckState(Qt.CheckState.Checked)
			highlight_checkbox.stateChanged.connect(
				lambda state, name=sonar_name: self.on_sonar_highlight_changed(name, state)
			)
			
			# Bouton pour afficher/masquer les détails
			toggle_button = QPushButton(f"Sonar: {sonar_name}")
			toggle_button.setCheckable(True)
			toggle_button.setChecked(False)
			
			header_layout.addWidget(highlight_checkbox)
			header_layout.addWidget(toggle_button, 1)  # Le bouton prend le reste de la place
			
			# Contenu (TextEdit) - caché par défaut
			info_textbox = QTextEdit()
			info_textbox.setReadOnly(True)
			info_textbox.setMinimumHeight(100)
			info_textbox.setMaximumHeight(150)
			info_textbox.setVisible(False)
			
			# Ajouter à la disposition principale
			group_layout.addWidget(header)
			group_layout.addWidget(info_textbox)
			
			# Connecter le bouton pour afficher/masquer
			toggle_button.clicked.connect(
				lambda checked, tb=info_textbox: tb.setVisible(checked)
			)
			
			# Stocker les références aux widgets
			self.sonar_analysis_widgets[sonar_name] = {
				'checkbox': highlight_checkbox,
				'textbox': info_textbox,
				'group': group,
				'button': toggle_button
			}
			
			# Ajouter le groupe au layout principal
			self.analysis_content_layout.insertWidget(
				self.analysis_content_layout.count() - 1,
				group
			)

	def set_bag_info(self, bag_path, bag_info):
		"""Set the bag file information and path."""
		
		self.bag_path = bag_path
		self.bag_info = bag_info
		for _, category in self.marker_categories.items():
			category.cleanup()
		self.marker_categories.clear()
		self.clear_all_curves()
		
		marker_maker = lambda: OvertakeMarkerDialog(self.current_time_marker.value(), self.bag_info.duration, self)()
		self.marker_categories['Dépassement'] = MarkerCategory("Dépassement", Qt.red, True, MarkerCategoryEnum.Overtake,
												marker_maker)
		self.marker_categories['Croisement'] = MarkerCategory("Croisement", Qt.magenta, True, MarkerCategoryEnum.Oncoming,
												marker_maker)

		self.load_sonar_data()

		self.base_dir = os.path.dirname(bag_path)
		self.dft_markers_path = os.path.join(self.base_dir, "marker_export.json")
		if os.path.exists(self.dft_markers_path):
			self.on_import_markers(self.dft_markers_path)

		# Auto-add sonar topics with default colors
		for sonar in self.sonar_datas:
			self.add_curve(sonar, False)
			self.peaks[sonar] = detect_peaks(self.sonar_datas[sonar].datas, 
				threshold_drop=3.0,
				min_duration_sec=0.3, 
				recovery_tolerance_sec=0.1,
				feedback_handler=self.results_text_edit.setText)
			
		self.handle_peak_detection_results(self.peaks)
		self.populate_sonar_list()
		self.update_sonar_analysis_tab()

	def clear_all_curves(self):
		"""Clear all curves from the plot."""
		self.plot_widget.clear()
		for _, category in self.marker_categories.items():
			category.visible = False
		self.curves = {}
		self.curve_list.clear()
		# Re-add the time marker after clearing
		self.current_time_marker = InfiniteLine(pos=0, angle=90, movable=False, pen='w')
		self.plot_widget.addItem(self.current_time_marker)
		# Re-draw any visible markers
	
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
		curve_item = self.plot_widget.plot(x_vals, y_vals, pen=sonar_data.color,
									name=topic,symbol='o', symbolSize=1,
									symbolBrush=sonar_data.color)
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
		for sonar, _ in self.sonar_datas.items():
			category_name = f"{sonar}_peaks"
			previous_peak_marker: Marker = self.current_peaks.get(category_name, None)
	
			if previous_peak_marker and ((previous_peak_marker.detail.start_time + previous_peak_marker.detail.duration) < time_seconds):
				if previous_peak_marker.category.visible == False:
					previous_peak_marker.hide()
				previous_peak_marker.clear_peak_visualization()
				self.current_peaks[category_name] = None
				self.update_sonar_info(sonar, "")
	
			if self.current_peaks.get(category_name, None) == None and self.sonar_datas[sonar].highlight == True:
				peak_marker = find_peak_in_category(self.marker_categories[category_name], time_seconds, 0.25)
				if peak_marker:
					peak_marker.show()
					peak_marker.display_peak_region()
					self.update_sonar_info(sonar, str(peak_marker))

				self.current_peaks[category_name] = peak_marker

	
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
			self.add_marker_category(type_name, color)

	def add_marker_category(self, name, color, visible=True, type: MarkerCategoryEnum = MarkerCategoryEnum.User):
		self.marker_categories[name] = MarkerCategory(name, color, visible, type)

	def remove_marker_category(self, name):
		self.previous_selected_marker = None
		if not name in self.marker_categories:
			return
		category: MarkerCategory = self.marker_categories[name]
		category.cleanup()
		del self.marker_categories[name]

	def on_remove_marker_category(self):
		"""Remove the selected marker type."""
		selected_items = self.marker_tree.selectedItems()
		if not selected_items:
			return
			
		for item in selected_items:
			# Only process top-level items (marker types)
			if item.parent() is None:
				type_name = item.text(0)
				self.remove_marker_category(type_name)
	
	def on_change_marker_color(self):
		"""Change the color of the selected marker type."""
		selected_items = self.marker_tree.selectedItems()
		if not selected_items:
			return
			
		for item in selected_items:
			# Only process top-level items (marker types)
			if item.parent() is None:
				category: MarkerCategory = item.category
				color = QColorDialog.getColor()
				if color.isValid():
					category.color = color.name()
					item.setText(1, color.name())
					category.visible = False
					category.visible = True
	
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
		
		timestamp, description, detail = self.marker_categories[type_name].new_marker_handler()
			
		# Add marker
		if timestamp:
			self.marker_categories[type_name].add_marker(timestamp, description, detail)
	
	
	def generic_add_marker_handler(self):
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
		
		return timestamp, description, None

	def overtake_add_marker_handler(self):
		current_time = self.current_time_marker.value()
		timestamp, ok = QInputDialog.getDouble(self, "Timestamp", 
											"Temps (secondes):", 
											current_time, 0, 
											self.bag_info.duration if self.bag_info else 999999, 2)
		if not ok:
			return
			
		car_color, ok = QInputDialog.getText(self, "Couleur", 
											"Couleur du véhicule:")
		
		#Remplacer car_type par une liste de choix contenant: "Voiture", "Camionnette", "Camion", "Bus", "Moto", "Vélo", "Autre"
		car_type, ok = QInputDialog.getText(self, "Type", 
											"Type du véhicule:")
		
		#Remplacer car_type par une liste de choix contenant: "Ligne nette", "Lignes éparses", "Amats de points", "Points isolés"
		lidar_shape = QInputDialog.getText(self, "Forme", 
											"Forme du nuage de points:")

		detail = {"type": car_type, "couleur": car_color, "forme": lidar_shape}
		description = f"{car_type} {car_color}"
		
		return timestamp, description, detail


	def on_remove_marker(self):
		"""Remove the selected marker."""
		self.previous_selected_marker = None
		selected_items = self.marker_tree.selectedItems()
		if not selected_items:
			return
			
		for item in selected_items:
			# Only process child items (markers)
			if item.parent() is not None:
				marker: Marker = item.marker
				category: MarkerCategory = marker.category
				category.remove_marker(marker)
	
	def marker_tree_item_changed(self, item, column):
		"""Handle changes in the marker tree."""
		if column == 2 and item.parent() is None:  # Visibility column for type
			category = item.category
			category.visible = (item.checkState(2) == Qt.Checked)
		if column == 1:
			color = item.marker.color if item.parent() != None else item.category.color
			item.setBackground(1, QBrush(QColor(color)))
	
	def marker_tree_item_selected(self):
		if self.previous_selected_marker:
			if self.previous_selected_marker.category.visible == False:
				self.previous_selected_marker.hide()
			self.previous_selected_marker.clear_peak_visualization()
		selected_item = self.marker_tree.selectedItems()
		if selected_item:
			item = selected_item[0]
			if item.parent() == None:
				return
			self.previous_selected_marker = item.marker
			item.marker.show()
			if item.marker.category.type == MarkerCategoryEnum.Peak:
				item.marker.display_peak_region()
			self.marker_details_text.setText(str(item.marker))

	def on_import_markers(self, dft_file_path = None):
		"""Import markers from a JSON file."""
		if not dft_file_path:
			file_path, _ = QFileDialog.getOpenFileName(
				self, "Importer des marqueurs", "", "Fichiers JSON (*.json)"
			)
		else:
			file_path = dft_file_path
	
		if not file_path:
			return
			
		try:
			with open(file_path, 'r') as f:
				data = json.load(f)
			
			# Import marker types and markers
			for category_data in data.get('marker_categories', []):
				type_name = category_data.get('name')
				if type_name:
					if not type_name in self.marker_categories:
						marker_category = MarkerCategory(
							type_name,
							category_data.get('color', '#FF0000'),
							category_data.get('visible', True),
							category_data.get('type', MarkerCategoryEnum.User)
						)
						self.marker_categories[type_name] = marker_category
					else:
						marker_category = self.marker_categories[type_name]

					# Import markers for this type
					for marker_data in category_data.get('markers', []):
						timestamp = marker_data.get('timestamp', 0.0)
						description = marker_data.get('description', '')
						detail = marker_data.get('detail', None)
						color = marker_data.get('color', None)
						display_label = marker_data.get('display_label', True)
						marker_category.add_marker(timestamp, description, detail, color,  display_label)
					
			if not dft_file_path:
				QMessageBox.information(self, "Import réussi", 
									f"Marqueurs importés avec succès depuis {file_path}")
								
		except Exception as e:
			QMessageBox.critical(self, "Erreur d'import", 
							f"Erreur lors de l'import des marqueurs: {str(e)}")
	
	def on_export_markers(self):
		"""Export markers to a JSON file."""
		export_categories = {k: v for k, v in self.marker_categories.items() if v.visible}

		if not export_categories:
			QMessageBox.information(self, "Aucun marqueur", 
								"Aucun marqueur à exporter.")
			return
		
		# Generate default filename with timestamp
		default_filename = self.dft_markers_path or f"markers_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
		
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
			
			for type_name, marker_category in export_categories.items():
				category_data = {
					'name': type_name,
					'color': marker_category.color,
					'visible': marker_category.visible,
					'type': marker_category.type,
					'markers': []
				}
				
				for _, marker in marker_category.markers.items():
					category_data['markers'].append({
						'timestamp': marker.stamp,
						'description': marker.description,
						'detail': marker.detail,
						'color': marker.color,
						'display_label': marker.display_label
					})
				
				data['marker_categories'].append(category_data)
			
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
			item.setCheckState(Qt.CheckState.Checked)
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
		self.peaks = results
		for topic, peaks in self.peaks.items():
			marker_category = f"{topic}_peaks"
			if marker_category in self.marker_categories:
				self.remove_marker_category(marker_category)
			self.add_marker_category(marker_category, self.sonar_datas[topic].color, False, MarkerCategoryEnum.Peak)
			category:MarkerCategory = self.marker_categories[marker_category]
			for peak in peaks:
				category.add_marker(peak.start_time, "{:.2f}".format(peak.mean), peak, None, False)
			results_text += f"{topic}: {len(peaks)} pics détectés\n"

		self.results_text_edit.setText(results_text)

	def on_sonar_highlight_changed(self, sonar_name: str, state: int):
		"""Gère le changement d'état de la checkbox pour mettre en évidence un sonar."""
		self.sonar_datas[sonar_name].highlight = (state == Qt.CheckState.Checked)

	def update_sonar_info(self, sonar_name, info: str):
		"""Met à jour la visibilité de l'information d'un sonar."""
		widgets = self.sonar_analysis_widgets.get(sonar_name, None)
		if widgets == None:
			return
		widgets['textbox'].setVisible(info != None)
		widgets['textbox'].setText(info if info != None else "")
		widgets['button'].click()

	def on_export_csv(self):
		pass

	def on_import_csv(self):
		pass
