from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
							QPushButton, QSlider, QLabel, QFileDialog, QTreeWidget, 
							QTreeWidgetItem, QComboBox, QCheckBox, QSizePolicy)
from PyQt5.QtWidgets import QTabWidget
from PyQt5.QtGui import QPixmap, QTransform, QColor
from rclpy.node import Node
from PyQt5.QtCore import Qt, QTimer
import os, tempfile, shutil, subprocess
from cyclosafe_player.src.BagReader import BagInfo, BagReader
from cyclosafe_player.src.GraphWidget import SonarGraphWidget
from rosidl_runtime_py.utilities import get_message

# Pour convertir CompressedImage en QImage pour l'affichage
def compressed_image_to_qimage(self, msg):
	import cv2
	import numpy as np
	from PyQt5.QtGui import QImage
	
	# Convertir les données compressées en tableau numpy
	np_arr = np.frombuffer(msg.data, np.uint8)
	img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	img_rgb = img
	# Convertir BGR en RGB
	# img_rgb = cv2.cvtColor(img, cv2.COLOR_RGB2RGB)
	
	# Créer une QImage
	height, width, channels = img_rgb.shape
	bytes_per_line = channels * width
	q_img = QImage(img_rgb.data, width, height, bytes_per_line, QImage.Format_BGR888)
	
	return q_img

class RosbagPlayerWindow(QMainWindow):

	colors = [QColor(255,0,0), QColor(0,255,0), QColor(0,0,255), QColor(255,255,0), QColor(0,255,255)]

	"""Main application window for rosbag player."""
	def __init__(self, node: Node, path: str = None):
		super().__init__()
		self.bag_path = None
		self.bag_reader = None
		self.bag_info = None
		self.is_playing = False
		self.selected_topics = set()
		self.publishers = {}  # Dictionnaire pour stocker les publishers par topic
		self.node = node  # Référence au nœud ROS 2
		self.working_path = None
		self.init_ui()
		if path:
			self.open_bag(path)
		
	def init_ui(self):
		self.setWindowTitle("ROS Bag Player")
		self.setGeometry(100, 100, 1200, 800)
		
		# Main widget and layout
		central_widget = QWidget()
		main_layout = QVBoxLayout()
		central_widget.setLayout(main_layout)
		self.setCentralWidget(central_widget)

		self.setFocusPolicy(Qt.StrongFocus)
		
		# Top controls
		top_controls = QHBoxLayout()
		
		# Open button
		self.open_button = QPushButton("Open Bag")
		self.open_button.clicked.connect(self.open_bag)
		top_controls.addWidget(self.open_button)
		
		# Playback controls
		self.play_button = QPushButton("Play")
		self.play_button.clicked.connect(self.toggle_playback)
		self.play_button.setEnabled(False)
		top_controls.addWidget(self.play_button)
		
		self.stop_button = QPushButton("Stop")
		self.stop_button.clicked.connect(self.stop_playback)
		self.stop_button.setEnabled(False)
		top_controls.addWidget(self.stop_button)
		
		# Playback speed
		speed_label = QLabel("Speed:")
		top_controls.addWidget(speed_label)
		
		self.speed_combo = QComboBox()
		for speed in ["0.1x", "0.25x", "0.5x", "1.0x", "2.0x", "5.0x", "10.0x"]:
			self.speed_combo.addItem(speed)
		self.speed_combo.setCurrentText("1.0x")
		self.speed_combo.currentTextChanged.connect(self.change_playback_speed)
		self.speed_combo.setEnabled(False)
		top_controls.addWidget(self.speed_combo)
		
		# Loop playback
		self.loop_checkbox = QCheckBox("Loop")
		self.loop_checkbox.setEnabled(False)
		top_controls.addWidget(self.loop_checkbox)
		
		# Add spacer
		top_controls.addStretch()
		
		# Add file info label
		self.file_info_label = QLabel("No file loaded")
		top_controls.addWidget(self.file_info_label)
		
		main_layout.addLayout(top_controls)
		
		# Main content area - two columns side by side
		main_content = QHBoxLayout()

		left_layout = QVBoxLayout()
		# Left column with tabs
		tab_widget = QTabWidget()

		# --- Tab 1: Topic list ---
		topic_list_widget = QWidget()
		topic_list_layout = QVBoxLayout()
		topic_list_widget.setLayout(topic_list_layout)

		self.topic_tree = QTreeWidget()
		self.topic_tree.setHeaderLabels(["Topics", "Type", "Count"])
		self.topic_tree.setColumnWidth(0, 300)
		self.topic_tree.itemChanged.connect(self.topic_selection_changed)
		topic_list_layout.addWidget(self.topic_tree)

		tab_widget.addTab(topic_list_widget, "Topics")

		left_layout.addWidget(tab_widget)
		# # --- Tab 2: Miniatures ---
		# image_tab = QWidget()
		# image_tab_layout = QVBoxLayout()
		# image_tab.setLayout(image_tab_layout)

		# scroll_area = QScrollArea()
		# scroll_area.setWidgetResizable(True)
		self.image_preview_container = QWidget()
		self.image_preview_layout = QVBoxLayout()
		self.image_preview_container.setLayout(self.image_preview_layout)
		left_layout.addWidget(self.image_preview_container)
		# scroll_area.setWidget(self.image_preview_container)

		# image_tab_layout.addWidget(scroll_area)
		# tab_widget.addTab(image_tab, "Miniatures")

		main_content.addLayout(left_layout, 1)

		# Right column - Timeline
		right_column = QVBoxLayout()
		right_column.addWidget(QLabel("Timeline:"))
		
		# Timeline in its own layout
		timeline_layout = QHBoxLayout()
		
		self.time_label = QLabel("0.00 s")
		timeline_layout.addWidget(self.time_label)
		
		self.timeline_slider = QSlider(Qt.Horizontal)
		self.timeline_slider.setEnabled(False)
		self.timeline_slider.valueChanged.connect(self.timeline_value_changed)
		self.timeline_slider.sliderPressed.connect(self.timeline_drag_started)
		self.timeline_slider.sliderReleased.connect(self.timeline_drag_ended)
		timeline_layout.addWidget(self.timeline_slider)
		
		self.duration_label = QLabel("0.00 s")
		timeline_layout.addWidget(self.duration_label)
		
		right_column.addLayout(timeline_layout)
		
		self.sonar_graph = SonarGraphWidget(self.node)
		right_column.addWidget(self.sonar_graph, 3)
		self.sonar_graph.init_tab_ui(tab_widget)

		# (Optionnel) Conserver un espace pour d'autres infos, par exemple :
		placeholder = QWidget()
		placeholder.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
		right_column.addWidget(placeholder)
		main_content.addLayout(right_column, 3)  # 3 est le stretch factor (donne plus d'espace)
		
		main_layout.addLayout(main_content)
		
		# Status bar
		self.status_bar = self.statusBar()
		self.status_bar.showMessage("Ready")
		
		# Timer for updating UI
		self.update_timer = QTimer()
		self.update_timer.timeout.connect(self.update_ui)
		self.update_timer.start(10)  # 50 ms update rate

	def open_bag(self, file_path: str = None):
		"""Open a bag file and load its contents, with support for zstd compression."""
		if file_path is None:
			file_dialog = QFileDialog()
			file_path, _ = file_dialog.getOpenFileName(self, "Open ROS Bag", os.path.join(os.getenv("HOME"), "data"))
		
		if not file_path:
			return
			
		self.status_bar.showMessage(f"Loading {file_path}...")
		
		# Check if the file is zstd compressed
		is_zstd = file_path.endswith('.zst') or file_path.endswith('.zstd')
		temp_dir = None
		
		try:
			if is_zstd:
				# Create temporary directory for decompressed bag
				temp_dir = tempfile.mkdtemp(prefix="rosbag_decompressed_")
				base_name = os.path.basename(file_path)
				decompressed_name = base_name.rsplit('.', 1)[0]  # Remove zst/zstd extension
				decompressed_path = os.path.join(temp_dir, decompressed_name)
				
				self.status_bar.showMessage(f"Decompressing {file_path}...")
				
				# Run zstd decompression
				try:
					result = subprocess.run(
						["zstd", "-d", "-o", decompressed_path, file_path],
						check=True,
						stdout=subprocess.PIPE,
						stderr=subprocess.PIPE
					)
					# Use the decompressed file for further processing
					self.working_path = decompressed_path
					self.status_bar.showMessage(f"Decompressed {file_path}, loading...")
				except subprocess.CalledProcessError as e:
					self.status_bar.showMessage(f"Error decompressing file: {e.stderr.decode()}")
					return
				except FileNotFoundError:
					self.status_bar.showMessage("Error: zstd command not found. Please install zstd.")
					return
			else:
				# Use the original file
				self.working_path = file_path
			
			# Store the original path
			self.bag_path = file_path
			
			# Extract bag information using the working path (decompressed or original)
			self.bag_info = BagInfo(self.working_path)
			
			# Update UI with bag info
			base_name = os.path.basename(file_path)
			duration_str = f"{self.bag_info.duration:.2f}s"
			msg_count = self.bag_info.message_count
			self.file_info_label.setText(f"{base_name} | {duration_str} | {msg_count} messages")
			
			# Set up timeline
			self.timeline_slider.setRange(0, int(self.bag_info.duration * 100))  # 100 steps per second
			self.timeline_slider.setValue(0)
			self.timeline_slider.setEnabled(True)
			self.duration_label.setText(f"{self.bag_info.duration:.2f} s")
			
			# Populate topic tree
			self.topic_tree.clear()
			for topic_name, topic_data in self.bag_info.topic_info.items():
				item = QTreeWidgetItem(self.topic_tree)
				item.setText(0, topic_name)
				item.setText(1, topic_data.msg_type)
				item.setText(2, str(topic_data.message_count))
				item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
				item.setCheckState(0, Qt.Checked)

			self.sonar_graph.set_bag_info(self.working_path, self.bag_info, self.bag_path)

			# Enable controls
			self.play_button.setEnabled(True)
			self.stop_button.setEnabled(True)
			self.speed_combo.setEnabled(True)
			self.loop_checkbox.setEnabled(True)
			
			# Keep reference to temp dir so we can clean it up later
			self.temp_dir = temp_dir if is_zstd else None
			
			self.status_bar.showMessage(f"Loaded {file_path}")
		
		except Exception as e:
			# Clean up temp directory in case of error
			if temp_dir and os.path.exists(temp_dir):
				shutil.rmtree(temp_dir)
			self.status_bar.showMessage(f"Error loading bag: {str(e)}")
			raise

	def cleanup_resources(self):
		"""Clean up temporary resources when closing or opening a new bag."""
		if hasattr(self, 'temp_dir') and self.temp_dir and os.path.exists(self.temp_dir):
			try:
				shutil.rmtree(self.temp_dir)
				self.temp_dir = None
			except Exception as e:
				print(f"Error cleaning up temporary files: {str(e)}")
	
	def toggle_playback(self):
		"""Toggle between play and pause states."""
		if not self.working_path:
			return
		
		if not self.is_playing:
			self.start_playback()
		else:
			self.pause_playback()
	
	def start_playback(self):
		"""Start or resume bag playback."""
		if not self.bag_reader:
			# Create and configure reader thread
			self.bag_reader = BagReader(self.working_path)
			self.bag_reader.message_read.connect(self.process_message)
			self.bag_reader.finished.connect(self.playback_finished)
			
			# Set playback speed
			speed_text = self.speed_combo.currentText()
			speed = float(speed_text.rstrip('x'))
			self.bag_reader.set_playback_speed(speed)
			
			# Start from current timeline position
			pos = self.timeline_slider.value() / 100.0
			self.bag_reader.seek(pos)
			
			self.bag_reader.start()
		else:
			# Resume paused playback
			self.bag_reader.resume()
		
		self.is_playing = True
		self.play_button.setText("Pause")
		self.status_bar.showMessage("Playing...")
	
	def pause_playback(self):
		"""Pause bag playback."""
		if self.bag_reader:
			self.bag_reader.pause()
			self.is_playing = False
			self.play_button.setText("Play")
			self.status_bar.showMessage("Paused")
	
	def stop_playback(self):
		"""Stop bag playback."""
		if self.bag_reader:
			self.bag_reader.stop()
			self.bag_reader = None
			self.is_playing = False
			self.play_button.setText("Play")
			self.timeline_slider.setValue(0)
			self.status_bar.showMessage("Stopped")
			
	
	def change_playback_speed(self, speed_text):
		"""Change the playback speed."""
		current_speed = float(speed_text.rstrip('x'))
		if self.bag_reader:
			self.bag_reader.set_playback_speed(current_speed)

	def timeline_drag_started(self):
		"""Pause the playback when dragging the timeline."""
		if self.is_playing:
			self.pause_playback()

	def timeline_drag_ended(self):
		"""Resume the playback when releasing the drag on the timeline."""
		if not self.is_playing:
			self.start_playback()


	def timeline_value_changed(self, value):
		"""Handle timeline slider value changes."""
		time_pos = value / 100.0
		self.time_label.setText(f"{time_pos:.2f} s")
		
		# If we're playing, seek to the new position
		if self.bag_reader:
			self.bag_reader.seek(time_pos)
	
	def topic_selection_changed(self, item, column):
		"""Handle topic selection changes."""
		if column == 0:
			topic_name = item.text(0)
			if item.checkState(0) == Qt.Checked:
				self.selected_topics.add(topic_name)
				
				# Créer un publisher pour ce topic
				if topic_name not in self.publishers and self.node:
					topic_type_str = self.bag_info.topic_info[topic_name].msg_type
					msg_type = get_message(topic_type_str)
					self.publishers[topic_name] = self.node.create_publisher(
						msg_type, 
						topic_name,
						10
					)
					self.node.get_logger().info(f"Publishing on {topic_name}")
					
					# Pour les CompressedImage, on peut encore créer un widget d'affichage
					if topic_type_str == 'sensor_msgs/msg/CompressedImage':
						pass
			else:
				self.selected_topics.discard(topic_name)
				# Détruire le publisher si existant
				if topic_name in self.publishers:
					self.node.destroy_publisher(self.publishers[topic_name])
					del self.publishers[topic_name]
					
	
	def process_message(self, topic, msg, timestamp):
		"""Process a message from the bag."""
		if topic in self.selected_topics and topic in self.publishers:
			# Publier le message
			self.publishers[topic].publish(msg)
			
			# Pour les CompressedImage, mettre à jour la miniature
			topic_type_str = self.bag_info.topic_info[topic].msg_type
			if topic_type_str == 'sensor_msgs/msg/CompressedImage':
				img = compressed_image_to_qimage(self, msg)
				
				# Rotation de 90° de l'image
				transform = QTransform().rotate(180)
				rotated_img = img.transformed(transform)

				pixmap = QPixmap.fromImage(rotated_img).scaledToWidth(400, Qt.SmoothTransformation)

				# Afficher ou mettre à jour l'image
				if not hasattr(self, 'image_label'):
					self.image_label = QLabel()
					self.image_label.setAlignment(Qt.AlignCenter)
					# self.image_label.setFixedWidth(200)
					self.image_preview_layout.addWidget(self.image_label)

				self.image_label.setPixmap(pixmap)
				self.image_label.setToolTip(f"{topic} @ {timestamp:.2f}s")

		
		# Update timeline
		self.timeline_slider.blockSignals(True)
		self.timeline_slider.setValue(int(timestamp * 100))
		self.timeline_slider.blockSignals(False)

	def playback_finished(self):
		"""Handle playback finished."""
		if self.loop_checkbox.isChecked():
			# Restart playback
			self.stop_playback()
			self.start_playback()
		else:
			self.is_playing = False
			self.play_button.setText("Play")
			self.bag_reader = None
			self.status_bar.showMessage("gpPlayback finished")
	
	def update_ui(self):
		"""Update UI elements periodically."""
		if self.is_playing and self.bag_reader:
			current_time = self.bag_reader.current_time
			self.time_label.setText(f"{current_time:.2f} s")
			
			# Update timeline without triggering valueChanged signals
			self.timeline_slider.blockSignals(True)
			self.timeline_slider.setValue(int(current_time * 100))
			self.timeline_slider.blockSignals(False)

			self.sonar_graph.update_current_time(current_time)

	def keyPressEvent(self, event):
		"""Handle keyboard shortcuts."""
		if event.key() == Qt.Key_Space:
			self.toggle_playback()
		elif event.key() == Qt.Key_Up:
			current_index = self.speed_combo.currentIndex()
			next_index = min(current_index + 1, self.speed_combo.count() - 1) 
			self.speed_combo.setCurrentIndex(next_index)
			self.change_playback_speed(self.speed_combo.currentText())
		elif event.key() == Qt.Key_Down:
			current_index = self.speed_combo.currentIndex()
			prev_index = max(current_index - 1, 0)
			self.speed_combo.setCurrentIndex(prev_index)
			self.change_playback_speed(self.speed_combo.currentText())
		elif event.key() == Qt.Key_Left:
			current_value = self.timeline_slider.value()
			self.timeline_slider.setValue(max(current_value - 100, 0))
		elif event.key() == Qt.Key_Right:
			current_value = self.timeline_slider.value()
			self.timeline_slider.setValue(min(current_value + 100, self.timeline_slider.maximum()))
