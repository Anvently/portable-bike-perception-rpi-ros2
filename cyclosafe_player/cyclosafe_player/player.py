#!/usr/bin/env python3
import sys
import rclpy
import threading
from PyQt5.QtWidgets import QApplication
import signal
from cyclosafe_player.src.PlayerWindow import RosbagPlayerWindow

# Ajoutez cette fonction pour gérer le signal SIGINT (Ctrl+C)
def signal_handler(sig, frame):
	print("Ctrl+C détecté, fermeture de l'application...")
	QApplication.quit()

def main(args=None):
	# Initialize ROS node
	rclpy.init()
	node = rclpy.create_node('rosbag_player')
	
	signal.signal(signal.SIGINT, signal_handler)
	print(args)
	executor = rclpy.executors.SingleThreadedExecutor()
	executor.add_node(node)
	ros_thread = threading.Thread(target=executor.spin, daemon=True)
	ros_thread.start()

	# Create and run Qt application
	app = QApplication(sys.argv)
	window = RosbagPlayerWindow(node, args[0])
	window.show()

	exit_code = app.exec_()
	
	# Cleanup
	if window.bag_reader:
		window.bag_reader.stop()
	window.cleanup_resources()
	rclpy.shutdown()
	sys.exit(exit_code)

def main_cli():
	custom_args = sys.argv[1:]
	
	# Séparer les arguments ROS des arguments personnalisés
	ros_args = []
	custom_only = []
	
	i = 0
	while i < len(custom_args):
		if custom_args[i] == '--ros-args':
			# Ajouter tous les arguments ROS à ros_args
			while i < len(custom_args) and not custom_args[i].startswith('--'):
				ros_args.append(custom_args[i])
				i += 1
		else:
			custom_only.append(custom_args[i])
			i += 1
	
	# Appeler main avec les arguments personnalisés
	main(custom_only)

if __name__ == "__main__":
	main()