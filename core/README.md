# Cyclosafe core 

Ce repo contient tous les packages et utilitaires nécessaires à la prise des mesures. Cela correspond à l'ensemble des noeuds et des scripts qui doivent tourner sur le raspberry.

## Installation

~~~
git clone [url] cyclosafe
cd cyclosafe
# Installation complète
./setup/install.sh

# Pour configurer les services seulement
./setup/systemd/setup_services.sh
~~~

## Usage

### Sourcer l'environnement
~~~
source ./setup/.bashrc
~~~

### Sourcer l'environnement à l'ouverture d'un nouveau terminal

~~~
$(cat ./bashrc) >> ~/.bashrc
~~~

### Pour compiler l'ensemble des packages

~~~
source ./setup/.bashrc
cy_core_build
~~~

### Démarrer la prise de mesure

~~~
ros2 launch cyclosafe cyclosafe.launch.py record:=false # (sans enregistrement)
ros2 launch cyclosafe cyclosafe.launch.py record:=true # (avec enregistrement)
~~~

### Activer/désactiver la prise de mesures avec enregistrement au démarrage du raspberry

~~~
sudo systemctl enable cyclosafed.service # Activer
sudo systemctl disable cyclosafed.service # Désactiver
~~~

## Documentation

### Détails sur les packages et les différents noeuds

[**cyclosafe.md**](cyclosafe.md)

### Détails sur l'installation

[**setup/README**](setup/README.md)

### Scripts

[**scripts/README**](scripts/README.md)

### Services systemd

[**setup/systemd/README**](setup/systemd/README.md)

### Documentation ROS 2

**Documentation officielle** :

https://docs.ros.org/en/jazzy/Tutorials.html

La suite de tutoriel basés sur **turtlesim** est une bonne façon d'appréhender les différents concepts de ROS : https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools.html


**Excellente source de tutoriels et compléments*** : 

https://roboticsbackend.com/category/ros2/