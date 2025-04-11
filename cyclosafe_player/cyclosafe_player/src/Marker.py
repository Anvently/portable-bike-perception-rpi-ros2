from PyQt5.QtWidgets import (QTreeWidget, QTreeWidgetItem)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from pyqtgraph import PlotWidget, InfiniteLine, mkPen
from enum import IntEnum

class MarkerCategoryEnum(IntEnum):
	User = 0
	Peak = 0

class MarkerCategory:
	"""Class to represent a type of marker with its color and visibility state."""

	tree_widget: QTreeWidget = None

	def __init__(self, name, color=Qt.red, visible=True, type=MarkerCategoryEnum.User):
		self.name = name
		self._color = color if isinstance(color, str) else color.name()
		self._visible = visible
		self.markers = []
		self.type = type
		if MarkerCategory.tree_widget:
			self._tree_insert_category_widget()
		
	def add_marker(self, stamp: float, description: str = "", detail = None,
				   color = None, display_label = True):
		if color == None:
			color = self._color
		marker = Marker(self, stamp, description, detail, color, self._visible, display_label)
		self.markers.append(marker)
		self.widget.setText(3, str(len(self.markers)))
		self._tree_insert_marker_widget(marker)
	
	def remove_marker(self, marker):
		if marker in self.markers:
			self.markers.remove(marker)
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
		for marker in self.markers:
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
		marker.widget = QTreeWidgetItem(self.widget)
		marker.widget.marker = marker
		marker.widget.setText(0, f"{marker.stamp:.2f}s")
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
		for marker in self.markers:
			marker.show() if value == True else marker.hide()

	@property
	def color(self):
		return self._color
	
	@color.setter
	def color(self, value):
		self._color = value
		for marker in self.markers:
			marker.setColor(value)

	def cleanup(self):
		for marker in self.markers:
			self.remove_marker(marker)
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
		Marker.plot_widget.addItem(self.line)
		self.line.marker = self
	
	def _plot_remove_line(self):
		if (Marker.plot_widget == None):
			raise Exception("No plot assigned to Marker")
		Marker.plot_widget.removeItem(self.line)
		self.line.marker = None
		self.line = None

	def show(self):
		self.visible = True
		if self.line == None:
			self._plot_insert_line()

	def hide(self):
		self.visible = False
		if self.line != None:
			self._plot_remove_line()

	def setColor(self, value):
		self.color = value
		self.line.setPen(mkPen(color=self.color, width=1, style=Qt.DashLine))
