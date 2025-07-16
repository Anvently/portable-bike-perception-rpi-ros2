# Cyclosafe core 

Ce repo contient tous les packages et utilitaires nécessaires à la prise des mesures. Cela correspond à l'ensemble des noeuds et des scripts qui doivent tourner sur le raspberry.

- [Cyclosafe core](#cyclosafe-core)
	- [Flasher la carte SD](#flasher-la-carte-sd)
	- [Installation (sans l'image du firmware)](#installation-sans-limage-du-firmware)
	- [Usage](#usage)
		- [Sourcer l'environnement](#sourcer-lenvironnement)
		- [Sourcer l'environnement à l'ouverture d'un nouveau terminal](#sourcer-lenvironnement-à-louverture-dun-nouveau-terminal)
		- [Pour compiler l'ensemble des packages](#pour-compiler-lensemble-des-packages)
		- [Démarrer la prise de mesure](#démarrer-la-prise-de-mesure)
		- [Activer/désactiver la prise de mesures avec enregistrement au démarrage du raspberry](#activerdésactiver-la-prise-de-mesures-avec-enregistrement-au-démarrage-du-raspberry)
	- [Documentation](#documentation)
		- [Détails sur les packages et les différents noeuds](#détails-sur-les-packages-et-les-différents-noeuds)
		- [Détails sur l'installation](#détails-sur-linstallation)
		- [Scripts](#scripts)
		- [Services systemd](#services-systemd)
		- [Documentation ROS 2](#documentation-ros-2)
	- [Installation hors-ligne](#installation-hors-ligne)
		- [Configurer le sous-réseau](#configurer-le-sous-réseau)


## Flasher la carte SD

> Suppose que vous vous êtes procuré l'image `cyclosafe_firmware.img.xz`.

Une fois la carte SD branché dans le PC, exécutez la commande suivante (remplacer `/dev/sdX` par le nom de la carte SD détectée par l'OS (généralement `/dev/sda`)):

~~~
xz -dc cyclosafe_firmware.img.xz | sudo dd of=/dev/sdX bs=4M status=progress && sync && sudo eject /dev/sdX
~~~

- `xz -dc cyclosafe_firmware.img.xz` : décompresse l'image
- `| sudo dd of=/dev/sdX bs=4M` : redirige l'image décompressée vers la carte SD. 
- `&& sync` : vérifie que tout a bien été écrit sur le disque
- `&& sudo eject /dev/sdX` : ejecte la carte SD qui peut désormais être arrachée.

Au premier démarrage de la carte sur un raspberry, la taille de la partition sera étendu automatiquement en fonction de l'espace disponible sur la carte SD et les mesures démarreront aussitôt.

## Installation (sans l'image du firmware)

> Suppose un accès internet sur le raspberry.
> Si le raspberry n'a pas accès à internet, privilégiez [**une installation hors ligne**](#installation-hors-ligne).

> Si vous avez flashé une carte SD avec le firmware de cyclosafe, vous pouvez ignorer cette section car tout y est déjà installé.
> 
> Sinon veillez à flasher une distribution **64 bits de Raspberry OS** (il est recommandé d'utiliser la version **lite** sans interface graphique).
>
> Il est recommandé de nommer l'utilisateur par défaut `cycliste`, par cohérence avec les exemples donnés dans la documentation.

~~~
git clone https://github.com/Anvently/Cyclosafe-firmware ~/cyclosafe
cd ~/cyclosafe/core
~~~


Installation complète
~~~
./setup/install.sh
~~~

Pour configurer les services seulement
~~~
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

## Installation hors-ligne

Si vous avez déjà accès au raspberry en ssh ou si le dossier core est déjà sur le raspberry, vous pouvez ignorer cette section et procéder à l'installation du [**core/**](./) sur le raspberry.

Autrement il est possible de procéder à une installation complètement vierge depuis l'hôte vers le raspberry.

### Configurer le sous-réseau

La première étape est de permettre la connexion ssh vers raspberry.

Pour cela suivez les [**instructions de configuration réseau**](../network.md), pour configurer les profils réseaux de l'hôte du raspberry (pour une installation complète il est aussi nécessaire que le raspberry ait accès à Internet).

Une fois le réseau configuré, vous pouvez simplement copier le dossier `core` vers le raspberry à l'aide de la commande suivante (remplacer `user` par l'utilisateur choisi lors de l'installation de raspberry OS) :

Depuis l'hôte :
~~~
scp -r ./core cyclist@192.168.2.2:/home/user/cyclosafe/core
~~~

Connectez-vous ensuite en ssh :

~~~
ssh cycliste@192.168.2.2
~~~

Vous avez désormais accès au raspberry.

Vous pouvez quitter la fenêtre ssh avec CTRL-D ou en fermant le terminal.

Pour poursuivre l'installation :

~~~
cd /home/$USER/cyclosafe/core
cat README.md
~~~

[**Voir les instructions**](#usage)