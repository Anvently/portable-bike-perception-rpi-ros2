# Résumé

A chaque capteur est associé un noeud ROS qui implémente éventuellement un driver et publie les mesures sur des topics dédiés.

Chaque noeud peut-être lancé indépendemment ou l'ensemble des noeuds sont lancés ensembles via la launch configuration [**cyclosafe.launch.py**](url)

# Structure

## cyclosafe

[**src/cyclosafe/README**](src/cyclosafe/README)

Noeud central du projet.

Contient :
- l'ensemble des noeuds de capteur écrits en python, à savoir :
	- noeud caméra (sensor_msgs/msg/CompressedImg)
	- noeud gps (cyclosafe_interfaces/msg/NavSatInfo)
	- 4 noeuds pour différents sonars (sensor_msgs/msg/Range)
	- noeud lidar (modèle waveshare no-name) (sensor_msgs/msg/Range)
- l'implémentation de classes utilisées par des noeuds ou d'autres packages :
	- ASerialSensor : modèle de noeud parsant des données envoyées via serial
	- ACamera : modèle de noeud caméra
	- Config : destinée à l'intégration d'un node dans une launch description, pour uniformiser la déclaration des paramètres à travers plusieurs launch description
- la [**launch description**](url) permettant de lancer l'ensemble des noeuds ensembles.

## cyclosafe_hub (obsolete)

[**src/cyclosafe_hub/README**](src/cyclosafe_hub/README)

Noeud en cpp qui a pour but d'aggréger des données venant de différents topics et de les enregistrer au format csv dans des fichiers isolés. Commande également l'enregistrement des photos via un service adressé au noeud camera.

Rendu obsolte par l'utilisation de rosbag, qui gère l'enregistrement, la sérialisation et la compression de toutes les données publiées.

## cyclosafe_interfaces

Définit les interfaces ROS 2 propres au projet, à savoir :
- ***cyclosafe_interfaces/msg/NavSatinfo*** : extension de sensor_msgs/msg/NavSatFix intégrant des données supplémentaires comme l'altitude, la vitesse, le nombre de satellites utilisées et la dop.
- ***cyclosafe_interfaces/srv/SaveImages (obsolte)*** : service permettant l'interaction entre le noeud hub et les noeuds caméras. Demande au noeud caméra d'enregistrer les images prises sur les X dernières secondes.

[**src/cyclosafe_interfaces/README**](src/cyclosafe_interfaces/README)

## cyclosafe_lidar

[**src/cyclosafe_lidar/README**](src/cyclosafe_lidar/README)

Package cpp qui permet d'intégrer les lidars de la gamme Benewake à ROS2. Implémente un driver via l'interface sériale et un noeud pour publier les données sous forme de sensor_msgs/msg/Range.

## cyclosafe_stl_ros2

[**src/cyclosafe_stl_ros2/README**](src/cyclosafe_stl_ros2/README)

Clone de https://github.com/Anvently/ldlidar_stl_ros2

Il s'agit d'un fork de https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2 afin de corriger une erreur de compilation liée à un **#include** manquant.

Implémente le driver du lidar STL-27 et l'intègre à ROS2.

## rplidar_ros2

[**src/rplidar_ros2/README**](src/rplidar_ros2/README)

Clone de https://github.com/CreedyNZ/rplidar_ros2

Implémente le driver du lidar rplidar-c1 et l'intègre à ROS2.
