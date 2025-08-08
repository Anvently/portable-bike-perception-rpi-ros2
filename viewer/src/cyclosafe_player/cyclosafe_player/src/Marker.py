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

from PyQt5.QtWidgets import (QTreeWidget, QTreeWidgetItem, QToolTip, QFormLayout, QVBoxLayout,
							 QDoubleSpinBox, QComboBox, QDialogButtonBox, QLineEdit, QDialog)
from PyQt5.QtCore import Qt, QPoint
from PyQt5.QtGui import QColor
from pyqtgraph import PlotWidget, InfiniteLine, mkPen, LinearRegionItem
from enum import IntEnum

class GenericMarkerDialog(QDialog):
	def __init__(self, current_time, max_time, instance = None, parent=None):
		super().__init__(parent)
		self.setWindowTitle("Ajouter un marqueur" if not instance else "Editer un marqueur")
		self.layout = QVBoxLayout()
		self.form = QFormLayout()

		self.time_input = QDoubleSpinBox()
		self.time_input.setRange(0, max_time)
		self.time_input.setDecimals(2)
		self.time_input.setValue(current_time if not instance else instance.stamp)
		self.form.addRow("Temps (secondes):", self.time_input)

		self.description_input = QLineEdit()
		self.description_input.setPlaceholderText("Description du marqueur")
		self.description_input.setText("" if not instance else instance.description)
		self.form.addRow("Description:", self.description_input)

		self.layout.addLayout(self.form)

		self.buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
		self.buttons.accepted.connect(self.accept)
		self.buttons.rejected.connect(self.reject)
		self.layout.addWidget(self.buttons)

		self.setLayout(self.layout)

	def get_data(self):
		timestamp = self.time_input.value()
		description = self.description_input.text() or f"Marqueur à {timestamp:.2f}s"
		return timestamp, description, None
	
	def __call__(self):
		if self.exec_() == QDialog.Accepted:
			return self.get_data()
		return None, None, None

class OvertakeMarkerDialog(GenericMarkerDialog):
	def __init__(self, current_time, max_time, title, instance = None, parent=None):
		super().__init__(current_time, max_time, instance, parent)
		self.setWindowTitle(title)

		self.distance = QDoubleSpinBox()
		self.distance.setRange(0, 5.0)
		self.distance.setDecimals(2)
		self.distance.setValue(1.5 if not instance else float((instance.detail['distance'])))
		self.form.addRow("Distance (mètres):", self.distance)

		self.color_input = QLineEdit()
		self.color_input.setText("" if not instance else instance.detail['color'])
		self.form.addRow("Couleur du véhicule:", self.color_input)

		self.type_input = QComboBox()
		self.type_input.addItems(["Voiture", "Camionnette", "Camion", "Bus", "Moto", "Vélo", "Autre"])
		self.type_input.setCurrentText("" if not instance else instance.detail['vehicle'])
		self.form.addRow("Type du véhicule:", self.type_input)

		self.shape_input = QComboBox()
		self.shape_input.addItems(["Ligne nette", "Lignes éparses", "Amats de points", "Points isolés"])
		self.shape_input.setCurrentText("" if not instance else instance.detail['shape'])
		self.form.addRow("Forme du nuage de points:", self.shape_input)

	def get_data(self):
		timestamp = self.time_input.value()
		description = self.description_input.text() or f"{self.type_input.currentText()} {self.color_input.text()}"
		detail = {
			"vehicle": self.type_input.currentText(),
			"color": self.color_input.text(),
			"shape": self.shape_input.currentText(),
			"distance": float(self.distance.text())
		}
		return timestamp, description, detail

class NumericTreeWidgetItem(QTreeWidgetItem):
	def __lt__(self, other):
		column = self.treeWidget().sortColumn()
		try:
			return float(self.text(column)) < float(other.text(column))
		except ValueError:
			return self.text(column) < other.text(column)

class MarkerCategoryEnum(IntEnum):
	User = 0
	Peak = 1
	Overtake = 2
	Oncoming = 3

class MarkerCategory:
	"""Class to represent a type of marker with its color and visibility state."""

	tree_widget: QTreeWidget = None
	dft_new_marker_handler = GenericMarkerDialog

	def __init__(self, name, color=Qt.red, visible=True, type=MarkerCategoryEnum.User,
			  new_marker_handler = None):
		self.name = name
		self._color = QColor(color).name() if not isinstance(color, str) else color
		self._visible = visible
		self.markers = {}
		self.type = type
		if MarkerCategory.tree_widget:
			self._tree_insert_category_widget()
		self.new_marker_handler = new_marker_handler or MarkerCategory.dft_new_marker_handler
		
	def add_marker(self, stamp: float, description: str = "", detail = None,
				   color = None, display_label = True) -> bool:

		"""If a marker with the same stamp already exists in the the category, it will not be added but not
		exception will be raised and False will be returned"""

		if color == None:
			color = self._color
		if stamp in self.markers:
			return False
		marker = Marker(self, stamp, description, detail, color, self._visible, display_label)
		self.markers[marker.stamp] = marker
		self.widget.setText(3, str(len(self.markers)))
		self._tree_insert_marker_widget(marker)
		return True

	def remove_marker(self, stamp):
		if stamp in self.markers:
			marker = self.markers[stamp]
			marker.hide()
			del self.markers[stamp]
			self._tree_remove_marker_widget(marker)
			self.widget.setText(3, str(len(self.markers)))

	def link_tree(tree: QTreeWidget):
		if (MarkerCategory.tree_widget != None):
			raise Exception("A tree was already assign to MarkerCategory")
		MarkerCategory.tree_widget  = tree

	def populate_tree(self):
		if (MarkerCategory.tree_widget == None):
			raise Exception("No tree assigned to MarkerCategory")
		if self.widget == None:
			self._tree_insert_category_widget()
		for _, marker in self.markers.items():
			self._tree_insert_marker_widget(marker)

	def _tree_insert_category_widget(self):
		if (MarkerCategory.tree_widget == None):
			raise Exception("No tree assigned to MarkerCategory")
		self.widget = QTreeWidgetItem(MarkerCategory.tree_widget)
		self.widget.category = self
		self.widget.setText(0, self.name)
		self.widget.setText(1, self._color)
		self.widget.setText(3, str(0))
		self.widget.setFlags(self.widget.flags() | Qt.ItemIsUserCheckable)
		self.widget.setCheckState(2, Qt.Checked if self._visible else Qt.Unchecked)
	
	def _tree_remove_category_widget(self):
		if (MarkerCategory.tree_widget == None):
			raise Exception("No tree assigned to MarkerCategory")
		MarkerCategory.tree_widget.invisibleRootItem().removeChild(self.widget)
		self.widget = None

	def _tree_insert_marker_widget(self, marker):
		if (MarkerCategory.tree_widget == None):
			raise Exception("No tree assigned to MarkerCategory")
		marker.widget = NumericTreeWidgetItem(self.widget)
		marker.widget.marker = marker
		marker.widget.setText(0, f"{marker.stamp:.2f}")
		marker.widget.setText(1, marker.description)
	
	def _tree_remove_marker_widget(self, marker):
		if (MarkerCategory.tree_widget == None):
			raise Exception("No tree assigned to MarkerCategory")
		if (marker.widget == None):
			return
		self.widget.removeChild(marker.widget)
		marker.widget = None

	@property
	def visible(self):
		return self._visible
	
	@visible.setter
	def visible(self, value: bool):
		self._visible = value
		for _, marker in self.markers.items():
			marker.show() if value == True else marker.hide()

	@property
	def color(self):
		return self._color
	
	@color.setter
	def color(self, value):
		self._color = value
		for _, marker in self.markers.items():
			marker.setColor(value)

	def cleanup(self):
		for _, marker in self.markers.items():
			marker.hide()
			self._tree_remove_marker_widget(marker)
		self.markers = {}
		if MarkerCategory.tree_widget and self.widget != None:
			self._tree_remove_category_widget()

class Marker:

	plot_widget:PlotWidget = None

	def link_plot(plot_widget: PlotWidget):
		if (Marker.plot_widget != None):
			raise Exception("A plot was already assign to Marker")
		Marker.plot_widget  = plot_widget

	def __init__(self, category: MarkerCategory, stamp: float,
			  description = "",  detail = None, color: QColor = None,
			  visible = True, display_label = False):
		self.stamp = stamp
		self.description = description
		self.detail = detail
		self.category = category
		self.color = category.color if not color else color
		self.widget = None
		self.visible = visible
		self.display_label = display_label
		self.line = None
		self.end_line = None
		self.region = None

		if Marker.plot_widget != None and self.visible:
			self._plot_insert_line()
		
	def _plot_insert_line(self):
		if (Marker.plot_widget == None):
			raise Exception("No plot assigned to Marker")
		if self.display_label:
			self.line: InfiniteLine = InfiniteLine(
				pos=self.stamp, 
				angle=90, 
				movable=False, 
				pen=mkPen(color=self.color, width=1, style=Qt.DashLine),
				label=self.description,
				labelOpts={'position': 0.9, 'color': self.color, 'fill': (0, 0, 0, 30)})
		else:
			self.line: InfiniteLine = InfiniteLine(
				pos=self.stamp, 
				angle=90, 
				movable=False, 
				pen=mkPen(color=self.color, width=1, style=Qt.DashLine))
		self.line.marker = self
		self.line.mouseClickEvent = lambda event: self.handle_click(event)
		Marker.plot_widget.addItem(self.line)
	
	def _plot_remove_line(self):
		if (Marker.plot_widget == None):
			raise Exception("No plot assigned to Marker")
		Marker.plot_widget.removeItem(self.line)
		self.line.marker = None
		self.line.mouseClickEvent = None
		self.line = None

	def show(self):
		self.visible = True
		if self.line == None:
			self._plot_insert_line()

	def hide(self):
		self.visible = False
		if self.line != None:
			self._plot_remove_line()
		self.clear_peak_visualization()

	def setColor(self, value):
		self.color = value
		self.line.setPen(mkPen(color=self.color, width=1, style=Qt.DashLine))

	def handle_click(self, event):
		marker_info = str(self)
		
		global_pos = event.screenPos().toPoint()
		QToolTip.showText(global_pos, marker_info)
		
		if self.category.type == MarkerCategoryEnum.Peak and self.detail is not None:
			self.display_peak_region()
		
		event.accept()

	def __str__(self) -> str:
		text = f"stamp={self.stamp}\ndescription={self.description}\n"
		if self.detail != None and hasattr(self.detail, "__str__"):
			text += str(self.detail)
		return (text)

	def display_peak_region(self):
		self.clear_peak_visualization()
		
		peak = self.detail
		
		if peak and hasattr(peak, 'duration') and peak.duration is not None:
			end_time = self.stamp + peak.duration
			
			self.end_line = InfiniteLine(
				pos=end_time,
				angle=90,
				movable=False,
				pen=mkPen(color=self.color, width=1, style=Qt.DashLine)
			)
			Marker.plot_widget.addItem(self.end_line)
			
			color_obj = QColor(self.color)
			color_obj.setAlpha(50) 
			
			self.region = LinearRegionItem(
				values=[self.stamp, end_time],
				brush=color_obj,
				movable=False
			)

			self.region.setBrush(color_obj)
			self.region.setZValue(-10)
			
			Marker.plot_widget.addItem(self.region)

	def clear_peak_visualization(self):
		"""Remove the end line and shaded region"""
		if self.end_line is not None:
			Marker.plot_widget.removeItem(self.end_line)
			self.end_line = None
		
		if self.region is not None:
			Marker.plot_widget.removeItem(self.region)
			self.region = None
