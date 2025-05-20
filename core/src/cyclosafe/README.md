# cyclosafe package

Noeud central du projet.

Contient :
- l'ensemble des noeuds de capteur écrits en python, à savoir :
	- [**noeud gps**](#gpspy) publiant des *cyclosafe_interfaces/msg/NavSatInfo*
	- 4 noeuds pour différents sonars  publiant des *sensor_msgs/msg/Range*
		- [**sonar**](#sonarpy) pour la gamme MaxBotix USB
		- [**sonar_sr04**](#sonar_sr04py) pour le modèle SR04
		- [**sonar_lv_pw**](#sonar_lv_pwpy-obsolete) pour la gamme MaxBotix en mesurant la largeur des pulse (PWM)
		- [**sonar_rs232**](#sonar_rs232py) pour la gamme MaxBotix LV10X0
	- [**tof_lidar**](#tof_lidarpy) pour le lidar unidirectionnel de Waveshare publiant des *sensor_msgs/msg/Range*
	- [**noeud caméra**](#camera_pipy) publiant des *sensor_msgs/msg/CompressedImage* sur le topic ***
- l'implémentation de classes utilisées par des noeuds ou d'autres packages :
	- [**ASerialSensor**](#aserialsensor) : modèle de noeud générique parsant des données envoyées via serial
	- [**ACamera**](#acamera) : modèle de noeud caméra0
	- [**Config**](#config) : destinée à l'intégration d'un node dans une launch description, pour uniformiser la déclaration des paramètres à travers plusieurs launch description
- la [**launch description**](#launch-configuration) permettant de lancer l'ensemble des noeuds ensembles.

# Noeuds executables

Chacun de ces noeuds peut-être executé independemment avec :

~~~
ros2 run cyclosafe [executable_sans_extension]
~~~

Pour lancer un noeud en précisant des paramètres :
~~~
ros2 run cyclosafe [executable] --ros-args [-p name1:=value -p name2:=value] [--log-level debug]
~~~

Pour débugger (si on utilise les fonctions debug de ROS) :
~~~
ros2 run cyclosafe [executable] --ros-args --log-level debug
~~~

Pour ajouter un namespace à l'executable (exemple: publier sur /lidar1/range au lieu de /range) :
~~~
ros2 run cyclosafe [executable] --ros-args -r __ns:=/lidar1
~~~

## gps.py

Hérite de ASerialPublisher.

Parse les frames NMEA envoyées par un serial device pour publier les informations suivantes sous forme de *cyclosafe_interfaces/msg/NavSatInfo* sur le topic **/gps**:

- **status** : no_fix, fix, fix (with GPS), fix (with GPS+Glonass), ...
- **latitude/longitude**: en format DDMM.MMMMM
- **altitude**: en mètres
- **hdop/pdop**
- **ground_speed**: en kilomètres/h
- **actives_sat**: nombre de satellites utilisés pour le fix

**Détail de l'interface** :

~~~
ros2 interface show cyclosafe_interfaces/msg/NavSatInfo
~~~

### Paramètres

> **port** : string
> - port serial sur lequel lire les données (défaut à */dev/ttyUSB0*)
> - **défaut** : */dev/ttyACM0*

> **baud** : int
> - baudrate du port serial 
> - **défaut** : *115200*

> **period** : float
> - interval auquel sont lues et parsées les données disponibles.
> - Seule la **première** donnée valide est prise en compte. Exemple : je lis des données envoyées à 10Hz sur un interval de 5Hz, j'aurais 5 données/secondes (et non pas 10 données réparties sur 5 lectures) et la position lue correspondra toujours à **la première position qui a pu être parsée** (pas nécessairement la dernière envoyéée par le capteur).
> - **défaut** : *0.5*
> - **unité** : secondes

> **start_time** : float
> - peut-être précisée dans le cadre d'une launch configuration pour référencer tous les noeuds sur un même départ. Non utilisée dans les faits car les timestamp attribués aux messages ne prennent pas compte de cette valeur.
> - **défaut** : *0.0*
> - **unité** : secondes depuis l'epoch

### Example

~~~
ros2 run cyclosafe gps --ros-args -p port:=/dev/ttyACM0 -p baud:=115200 -p period:=0.5

ros2 topic echo /gps
~~~

## sonar.py

Hérite de ASerialPublisher

Parse des distances lues sur une interface seriale sous la forme "RXXXX", et les publie sous forme de *sensor_msgs/msg/range* sur le topic **/range**.

### Paramètres

> **port** : string
> - port serial sur lequel lire les données (défaut à */dev/ttyUSB0*)
> - **défaut** : */dev/ttyUSB0*

> **baud** : int
> - baudrate du port serial 
> - **défaut** : *57600*

> **period** : float
> - interval auquel sont lues et parsées les données disponibles.
> - Seule la **dernière** donnée valide est prise en compte. Exemple : je lis des données envoyées à 10Hz sur un interval de 5Hz, j'aurais 5 données/secondes (et non pas 10 données réparties sur 5 lectures) et la distance lue correspondra toujours à **la distance la plus récente**.
> - **défaut** : *0.5*
> - **unité** : secondes

> **start_time** : float
> - peut-être précisée dans le cadre d'une launch configuration pour référencer tous les noeuds sur un même départ. Non utilisée dans les faits car les timestamp attribués aux messages ne prennent pas compte de cette valeur.
> - **défaut** : *0.0*
> - **unité** : secondes depuis l'epoch

> **unit** : string
> - Peut-être utilisé pour préciser une unité.
> - **accepte** : *mm*, *m*, *in* (inch)
> - **défaut** : *mm*

### Example

~~~
ros2 run cyclosafe sonar --ros-args -r __ns:=/sonar1 -p port:=/dev/ttyUSB0 -p baud:=57600 -p period:=0.2 -p unit:=mm

ros2 topic echo /sonar1/range
~~~


## sonar_sr04.py

Driver pour sonar SR04.

Publie les distances sous forme de *sensor_msgs/msg/range* sur le topic **/range**.

Déclenche une mesure et détermine la distance à partir de la largeur de l'onde carrée renvoyée par le sonar.

Par défaut, utilise **GPIO_6** comme pin pour déclencher la mesure et **GPIO_21** comme pin pour mesurer l'onde.

La vitesse du son utilisée est de *340m/s*, soit la vitesse du son dans l'air à 15°C.

### Paramètres

> **period** : float
> - interval entre chaque mesure
> - **défaut** : *0.1*
> - **unité** : secondes

> **start_time** : float
> - peut-être précisée dans le cadre d'une launch configuration pour référencer tous les noeuds sur un même départ. Non utilisée dans les faits car les timestamp attribués aux messages ne prennent pas compte de cette valeur.
> - **défaut** : *0.0*
> - **unité** : secondes depuis l'epoch

### Example

~~~
ros2 run cyclosafe sonar_sr04 --ros-args -r __ns:=/sonar2 -p period:=0.1

ros2 topic echo /sonar2/range
~~~


## sonar_rs232.py

Hérite de ASerialPublisher

Parse des distances lues sur une interface seriale sous la forme "RXXX", et les publie sous forme de *sensor_msgs/msg/range* sur le topic **/range**.

La seule différence avec [**sonar.py**](#sonarpy) est que le nombre de caractères à parser est de 4 (format *RXXX* au lieu de *RXXXX*).

Ce script est adapté à la lecture des données envoyées par la gamme de sonars MaxBotix LV10X0, qui envoie 4 caractères en rs232 (qu'il faut convertir en sérial via la puce MAX232). Les valeurs envoyées par cette gamme sont en pouces (*inch*) et le paramètre **unit** hérité de ASerialSensor est donc configuré (et non modifiable) à **in**.

### Paramètres

> **port** : string
> - port serial sur lequel lire les données (défaut à */dev/ttyUSB0*)
> - **défaut** : */dev/ttyUSB0*

> **baud** : int
> - baudrate du port serial 
> - **défaut** : *57600*

> **period** : float
> - interval auquel sont lues et parsées les données disponibles.
> - Seule la **dernière** donnée valide est prise en compte. Exemple : je lis des données envoyées à 10Hz sur un interval de 5Hz, j'aurais 5 données/secondes (et non pas 10 données réparties sur 5 lectures) et la distance lue correspondra toujours à **la distance la plus récente**.
> - **défaut** : *0.5*
> - **unité** : secondes

> **start_time** : float
> - peut-être précisée dans le cadre d'une launch configuration pour référencer tous les noeuds sur un même départ. Non utilisée dans les faits car les timestamp attribués aux messages ne prennent pas compte de cette valeur.
> - **défaut** : *0.0*
> - **unité** : secondes depuis l'epoch
> 

### Example

~~~
ros2 run cyclosafe sonar_rs232 --ros-args -r __ns:=/sonar3 -p port:=/dev/ttyAMA2 -p baud:=57600 -p period:=0.2

ros2 topic echo /sonar3/range
~~~

## sonar_lv_pw.py (obsolete)

Driver pour sonar LV10X0 en mode PWM.

Publie les distances sous forme de *sensor_msgs/msg/range* sur le topic **/range**.

Déclenche une mesure et détermine la distance à partir de la largeur de l'onde carrée renvoyée par le sonar.

Par défaut, utilise **GPIO_26** comme pin pour déclencher la mesure et **GPIO_19** comme pin pour mesurer l'onde.

L'objectif de ce noeud est de pouvoir utiliser la gamme MaxBotix LV10X0 sans passer par une puce MAX232, en mesurant la largeur des ondes carrées envoyées par la sortie PW du sonar.

Les tests effectués sont non concluant, les mesures étant très instables et imprécises. Il faudrait regarder à l'oscilloscope pour déterminer si le problème vient du code, du raspberry ou du sonar.

### Paramètres

> **period** : float
> - interval entre chaque mesure
> - **défaut** : *0.1*
> - **unité** : secondes
> - **min** : 0.038
> 	- valeur théorique minimum afin de permettre à l'onde envoyée de faire le chemin inverse sur une distance de 6.56 mètres.

> **start_time** : float
> - peut-être précisée dans le cadre d'une launch configuration pour référencer tous les noeuds sur un même départ. Non utilisée dans les faits car les timestamp attribués aux messages ne prennent pas compte de cette valeur.
> - **défaut** : *0.0*
> - **unité** : secondes depuis l'epoch

### Example

~~~
ros2 run cyclosafe sonar_lv_pw --ros-args -r __ns:=/sonar4 -p period:=0.1

ros2 topic echo /sonar4/range
~~~

## tof_lidar.py

Driver pour [**TOF Laser Range Sensor (Waveshare)**](https://www.waveshare.com/product/modules/sensors/light-ranging/tof-laser-range-sensor-c.htm)

Code basé sur les exemples donnés par le fabricant.

Parse les frames de 16 bytes envoyées par le capteur, en extrait les distance et les publie sous forme de  *sensor_msgs/msg/range* sur le topic **/range**.

Les valeurs nulles (*distance=0*) ou ayant une intensité retour nulle (*strength=0*) sont publiées comme étant **NaN**.

Le capteur envoie les frames à 100Hz et cette fréquence n'est pas paramètrable, la fréquence de publication des messages correspond donc à la fréquence à laquelle sont parsées les frames, plutôt qu'à celle à laquelle les données sont reçues.

Implémente 2 mode de decoding différent :
- **active decoding** : polling permanent sur la sortie sériale, sans *sleep()* et les bytes sont lus 1 par 1. Théoriquement le mode permettant la plus faible latence car les données sont envoyées en permanence, potentiellement moins performant en raison de l'absence de *sleep()* et de *read()* de 1.
- **trigger_decoding** : mesure cadencée par un timer ROS réglé sur le paramètre **period**. A chaque mesure, l'interface sériale est *flush()*, une commande de mesure est envoyée au lidar et la réponse du capteur est lue. Beaucoup plus performant mais peut générer une latence et limiter la fréquence puisque le capteur ne déclenche les mesures que lorsqu'il reçoit une commande.

Dans les faits et au moment des tests, bien que le code actuel utilise la fonction de decoding actif, il le fait au sein de la routine du timer (afin de respecter la période donnée en paramètre) et correspond donc à un bon compromis entre performance et faible latence.

### Paramètres

> **port** : string
> - port serial sur lequel lire les données (défaut à */dev/ttyUSB0*)
> - **défaut** : */dev/ttyUSB0*

> **baud** : int
> - baudrate du port serial 
> - **défaut** : *921600*

> **period** : float
> - interval auquel sont lues et parsées les données disponibles.
> - Seule la **dernière** donnée valide est prise en compte. Exemple : je lis des données envoyées à 10Hz sur un interval de 5Hz, j'aurais 5 données/secondes (et non pas 10 données réparties sur 5 lectures) et la distance lue correspondra toujours à **la distance la plus récente**.
> - **défaut** : *0.05*
> - **unité** : secondes

> **start_time** : float
> - peut-être précisée dans le cadre d'une launch configuration pour référencer tous les noeuds sur un même départ. Non utilisée dans les faits car les timestamp attribués aux messages ne prennent pas compte de cette valeur.
> - **défaut** : *0.0*
> - **unité** : secondes depuis l'epoch

### Example

~~~
ros2 run cyclosafe sonar --ros-args -r __ns:=/lidar1 -p port:=/dev/ttyS0 -p baud:=921600 -p period:=0.05

ros2 topic echo /lidar1/range
~~~

## camera_pi.py

Hérite de ACamera.

Utilise **picamera2** pour capturer les images.

Utilise **opencv2** pour compresser les images en *JPEG*.

Noeud permettant de capturer et publier des images sous forme de *sensor_msgs/msg/CompressedImage* sur le topic **/images/compressed**.

La caméra est configurée en mode vidéo avec la configuration donnée en paramètre. Le format utilisé est RGB888.

> **queue_size** : int
> - Définit la taille du *deque* qui met en cache les images pour un éventuel traitement ou enregistrement ultérieur. Si définit à **0**, aucune image n'est mise en cache.
> - **défaut** : *0*
> - **unité** : nombres d'images

> **resolution** : [int, int]
> - définit la résolution *[largeur, hauteur]* des images capturées.
> - **défaut** : *[800, 600]*
> - **unité** : px

> **interval** : float
> - interval auquel les images sont prises
> - **défaut** : *0.5*
> - **unité** : secondes

> **compression** : int
> - définit l'indice de qualité de la compression JPEG : 100 correspondant à la meilleure qualité.
> - **défaut** : *95*
> - **unité** : indice de 0 à 100

> **period** : float
> - interval auquel sont lues et parsées les données disponibles.
> - Seule la **dernière** donnée valide est prise en compte. Exemple : je lis des données envoyées à 10Hz sur un interval de 5Hz, j'aurais 5 données/secondes (et non pas 10 données réparties sur 5 lectures) et la distance lue correspondra toujours à **la distance la plus récente**.
> - **défaut** : *0.05*
> - **unité** : secondes

> **preview** : boolean
> - détermine si une preview de la caméra doit être activé au démarrage. Fonctionne seulement avec la console physique du raspberry et pas avec les pseudo-ttys (ssh, screen command, etc...).
> - **défaut** : False

> **start_time** : float
> - peut-être précisée dans le cadre d'une launch configuration pour référencer tous les noeuds sur un même départ. Utilisé par la fonction save_files de ACamera pour nommer les images sur un temps relatif au début de l'enregistrement.
> - **défaut** : *0.0*
> - **unité** : secondes depuis l'epoch

## camera_webcam.py (obsolete)

Hérite de ACamera.

Utilise **opencv2** pour capturer les images.

Utilise **opencv2** pour compresser les images en *JPEG*.

Noeud permettant de capturer des images sur une webcam traditionnelle (d'un laptop) et publier des images sous forme de *sensor_msgs/msg/CompressedImage* sur le topic **/images/compressed**.

L'origine de ce noeud était de faciliter les tests initiaux en testant la classe ACamera directement depuis l'host. 

Obsolete car :
- le **core** du projet n'est plus destiné à être installé sur l'host. 
- pas testé depuis un certain nombre de modifs sur la classe ACamera

> **queue_size** : int
> - Définit la taille du *deque* qui met en cache les images pour un éventuel traitement ou enregistrement ultérieur. Si définit à **0**, aucune image n'est mise en cache.
> - **défaut** : *0*
> - **unité** : nombres d'images

> **resolution** : [int, int]
> - définit la résolution *[largeur, hauteur]* des images capturées.
> - **défaut** : *[800, 600]*
> - **unité** : px

> **interval** : float
> - interval auquel les images sont prises
> - **défaut** : *0.5*
> - **unité** : secondes

> **compression** : int
> - définit l'indice de qualité de la compression JPEG : 100 correspondant à la meilleure qualité.
> - **défaut** : *95*
> - **unité** : indice de 0 à 100

> **period** : float
> - interval auquel sont lues et parsées les données disponibles.
> - Seule la **dernière** donnée valide est prise en compte. Exemple : je lis des données envoyées à 10Hz sur un interval de 5Hz, j'aurais 5 données/secondes (et non pas 10 données réparties sur 5 lectures) et la distance lue correspondra toujours à **la distance la plus récente**.
> - **défaut** : *0.05*
> - **unité** : secondes

> **preview** : boolean
> - détermine si une preview de la caméra doit être activé au démarrage. Fonctionne seulement avec la console physique du raspberry et pas avec les pseudo-ttys (ssh, screen command, etc...).
> - **défaut** : False

> **start_time** : float
> - peut-être précisé dans le cadre d'une launch configuration pour référencer tous les noeuds sur un même départ. Utilisé par la fonction save_files de ACamera pour nommer les images sur un temps relatif au début de l'enregistrement.
> - **défaut** : *0.0*
> - **unité** : secondes depuis l'epoch

# Classes

## ASerialSensor

## ACamera

Fournit une base commune à des noeuds caméras utilisant des librairies différentes pour capturer les images.

### Méthodes abstraites

> **init_camera()**
> - appelée à l'initialisation de la classe
> - générer des exceptions en cas d'erreur

> **capture() -> numpy.ndarray**
> - prend une photo et la retourne sous forme d'un numpy array qui peut-être donné en paramètre à *cv2.imencode()*
> - pour les images en niveaux de gris, retourne un tableau 2D
> - pour les images en couleur, retourne un tableau 3D au format BGR

### Paramètres

- **queue_size**
- **resolution**
- **interval**
- **compression**
- **preview**
- **start-time**

### cyclosafe_interfaces/srv/SaveImages

ACamera expose le noeud à l'utilisation du service *cyclosafe_interfaces/srv/SaveImages*.

Utilisé en conjonction avec un cache d'images **queue_size > 0**. Permet de demander au noeud l'enregistrement vers des fichiers des images prises dans les X dernières secondes.




## Config

# Launch configuration


