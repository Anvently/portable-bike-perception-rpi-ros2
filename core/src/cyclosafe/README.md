# cyclosafe package

Package python central du projet.

Contient :
- l'ensemble des noeuds de capteur écrits en python, à savoir :
	- [**noeud gps**](#gpspy) publiant des *cyclosafe_interfaces/msg/NavSatInfo*
	- [**noeud caméra**](#camera_pipy) publiant des *sensor_msgs/msg/CompressedImage* sur le topic ***
- l'implémentation de classes utilisées par des noeuds ou d'autres packages :
	- [**ASerialSensor**](#aserialsensor) : modèle de noeud générique parsant des données envoyées via serial
	- [**ACamera**](#acamera) : modèle de noeud caméra0
- la [**launch description**](#launch-description) permettant de lancer l'ensemble des noeuds ensembles.

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
> - peut-être précisée dans le cadre d'une launch description pour référencer tous les noeuds sur un même départ. Non utilisée dans les faits car les timestamp attribués aux messages ne prennent pas compte de cette valeur.
> - **défaut** : *0.0*
> - **unité** : secondes depuis l'epoch

### Example

~~~
ros2 run cyclosafe gps --ros-args -p port:=/dev/ttyACM0 -p baud:=115200 -p period:=0.5

ros2 topic echo /gps
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
> - peut-être précisée dans le cadre d'une launch description pour référencer tous les noeuds sur un même départ. Utilisé par la fonction save_files de ACamera pour nommer les images sur un temps relatif au début de l'enregistrement.
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
> - peut-être précisé dans le cadre d'une launch description pour référencer tous les noeuds sur un même départ. Utilisé par la fonction save_files de ACamera pour nommer les images sur un temps relatif au début de l'enregistrement.
> - **défaut** : *0.0*
> - **unité** : secondes depuis l'epoch

# Classes

## ASerialSensor

Fournit une base commune à tous les noeuds ayant pour fonctionnement de parser des données envoyées via une interface sériale.

### Méthodes abstraites

> **publish(self, data: Any)**
> - appelée lorsqu'une donnée complète a été parsée
> - doit construire et publier le message sur le topic correspondant
> - peut générer des exceptions

> **parse(self, data: Any) -> Any | None**
> - parse le contenu de self.buffer (qui contient la totalité des données sérielles lues jusqu'ici) afin d'en extraire une donnée pertinente.
> - retourne la donnée en question si elle est complète (self.buffer sera alors réinitialisé)
> - retourne None si aucune donnée n'a pu être parsée (self.buffer sera inchangé)
> - peut générer des exceptions

### Paramètres

- **port**
- **baud**
- **period**
- **start_time**
- **unit**

### Gestion des erreurs au runtime

Gère automatiquement les erreurs de parsing et/ou d'interface sériales.

Lorsque l'interface sériale n'est plus saine, la période de lecture est temporairement réglée sur 10s jusqu'à ce que la connexion soit rétablie.

Lors d'erreur lié au parsing, la période de lecture est temporairement réglée sur 3s jusqu'à la prochaine extraction de donnée complète.

Les logs du noeud peuvent être consultées afin de détecter les sources d'erreur.

## ACamera

Fournit une base commune à des noeuds caméras utilisant des librairies différentes pour capturer les images.

### Méthodes abstraites

> **init_camera(self)**
> - appelée à l'initialisation de la classe
> - générer des exceptions en cas d'erreur

> **capture(self) -> numpy.ndarray**
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

Utilisé en conjonction avec un cache d'images **queue_size > 0**. Permet de demander au noeud l'enregistrement vers des fichiers des images prises dans les X dernières millisecondes.

<ins>**Exemple**</ins> :

~~~
ros2 run cyclosafe camera_pi --ros-args -p queue_size:=200 -p interval:=0.5
~~~
Dans un autre terminal :
~~~
ros2 service call /save_images cyclosafe_interfaces/srv/SaveImages '{time: 1000, path: /home/npirard/Downloads}' 
~~~
Enregistre les images prises pendant la dernière seconde vers */home/npirard/Downloads*

<ins>**Résultat</ins>** :
~~~
requester: making request: cyclosafe_interfaces.srv.SaveImages_Request(time=1000, path='./Downloads')

response:
cyclosafe_interfaces.srv.SaveImages_Response(result=0)
~~~

Le nom de chaque image correspond au timestamp (au moment de la prise de la photo) relatif au paramètre **start_time** du noeud. Ainsi si l'image une image est prise *31.5s* après le début des enregistrements, elle se nommera *out_path/31500.jpg*.

La propriété **result** de la réponse indique si l'opération a échoué ou non:
- **result=0** : succès
- **result=1** : succès partiel, certaines images ont été enregistrées mais le cache n'était pas suffisamment grand pour respecter la contrainte de temps donné par **time**.
- **result=2** : échec, voir les logs du noeud camera pour le détail de l'erreur

Si une image a déjà été enregistrée dans le dossier donné en paramètre, elle n'est pas enregistrée en double.

# Launch description

C'est le script responsable de la déclaration de l'ensemble des noeuds à lancer et de leur paramètres.

Voir la documentation associée aux launch descriptions sur ROS2 :

https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html

Ou : https://roboticsbackend.com/ros2-launch-file-example/

## Usage

~~~
ros2 launch cyclosafe cyclsoafe.launch.py record:=true save:=false
~~~

## Paramètres

> **config** : str
> - défaut: **"""**
> - précise un chemin vers une config personnalisée à lancer
> - par défaut, utilise le fichier **config.py** situé dans le même dossier que la launch description.

> **record** : str
> - défaut: **"false""**
> - si **true**, le programme **rosbag record** sera lancé et l'ensemble des messages publiés par les noeuds seront enregistrés dans un rosbag.

> **out_path** : str
> - défaut: **""**
> - indique le répertoire d'enregistrement du dossier
> - si **""**,  le répertoire utilisé correspond à la variable d'environnement **$CYCLOSAFE_RECORD**
> - le nom du dossier contenant les enregistrement sera: ***out_path/YYYYMMDD-HHMMSS***

> **save** : str
> - défaut: **"false""**
> - si **true**, le noeud [**hub**](../cyclosafe_hub/README.md) sera démarré et enregistrera les mesures dans des fichiers isolés au format CSV.
> - **Obsolete** : non mis à jour depuis longtemps
> 	- gère les données publiées sous forme de *sensors_msgs/msg/range* et les images
> 	- ne gère que les données gps publiées sous forme de *sensor_msgs/msg/NavSatFix* (et non de *cyclosafe_interfaces/msg/NavSatInfo*)
> 	- contrairement au rosbag, ne compresse pas les données

> **log_level** : str
> - défaut: **"info""**
> - permet d'indiquer le niveau de log pour l'ensemble des noeuds ROS. Peut-être surchargé par l'argument **log_level** de la classe Sensor
> - **Valeurs possibles**:
> 	- debug
> 	- info
> 	- warning
> 	- error
> 	- fatal

## Spécificités

La structure de [**cyclosafe.launch.py**](launch/cyclosafe.launch.py) n'est pas tout à fait standard dans la mesure où :
- Elle repose lourdement sur la déclaration préalable d'une liste globale de [**Sensor**](../cyclosafe_config/README.md#classe-sensor) importée depuis un fichier extérieur ([**config.py**](launch/config.py)).
  
  L'avantage est de centraliser tout ce qui peut varier d'un test à l'autre sur un seul fichier. On peut par exemple avoir un fichier **config1.py** et **config2.py** qui correspond à un certain prototype avec des capteurs spécifiques ou des paramètres différents.
  
  Si on veut ensuite visualiser les données prises par la **config1** et celle par la **config2**, on peut le faire simplement en précisant à **cyclosafe_viewer** la config adéquate.

- Elle utilise une **fonction opaque** pour résoudre la description au runtime.

  En effet l'essentiel de la description dépend des paramètres donnés ou du contenu du fichier [**config.py**](launch/config.py). Or une launch description correspondant en quelque sorte à une recette de lancement qui n'est pas paramètrable directement.

  L'utilisation de **OpaqueFunction()** permet de déclarer une fonction qui sera executé avec le contexte du runtime avant de de renvoyer la description définitive.

### Résumé des étapes de lancement

1. **Execution de la fonction opaque launch_setup()** avec le contexte (cad. les arguments transmis avec la commande **ros2 launch cyclosafe cyclosafe.launch.py [...args])**
2. **Résolution des arguments**
3. **Prise du start_time**, valeur qui sera transmise en paramètre à chacun des noeuds lancés
4. **Optionnel : lancement de l'enregistrement du rosbag** si l'argument **record** est à **true**
5. **Optionnel : lancement du noeud hub** si l'argument **save** est à **true**
6. **Lancement de chaque noeud** à partir de la liste des capteurs importée depuis **config.py**
   - les noeuds avec **enable=false** sont ignorés
   - **start_time** est transmis à chaque noeud
   - un délai est éventuellement ajouté en fonction de l'agument **delay**
   - le **log_level** associé au noeud est ajusté sur celui spécifique au noeud, ou à défaut celui de la config
7. La description finale est renvoyée pour être lancée





