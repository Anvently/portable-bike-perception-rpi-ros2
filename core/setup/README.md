# Scripts d'installation et de configuration de l'environnement

Ce dossier contient :

- les scripts d'installation de l'environnement de travail :
	- [**ros_install.sh**](#ros_installsh)
	- [**install.sh**](#installsh)
	- [**systemd/setup_services.sh**](#systemdsetup_servicessh)
	- [**ip_config.sh**](#ip_configsh)
- le fichier [**.env**](#env) définissant les variables d'environnement nécessaires au projet.
- les fichiers [**config.txt**](#configtxt) et [**cmdline.txt**](#cmdlinetxt) indiquant les modifications à faire sur la partition boot pour activer/désactiver les différentes interfaces hardware
- le fichier [**.bashrc**](#bahrc) permettant de sourcer l'environnement cyclosafe

# Usage

Dans la plupart des cas vous pouvez simplement lancer le script **install.sh**.

~~~
chmod +x ./install.sh
./install.sh
~~~

Si vous souhaitez seuelement installer ROS, lancez le script **ros_install.sh**.

~~~
./ros_install.sh
~~~

Si vous souhaitez seulement mettre en place les services systemd, lancez le script **systemd/setup_services.sh**

~~~
./systemd/setup_services.sh
~~~

## Scripts d'installation

L'objectif de ces différents scripts est de faciliter et d'automatiser l'installation de toutes les dépendances et l'environnement nécessaires.

### install.sh

Peut-être lancé à partir d'une installation vierge ou partielle.

- si besoin, modifie les fichiers /boot/firmware/config.txt et /boot/firmware/cmdline.txt afin d'activer les overlays nécessaires au projet et de désactiver l'ipv6.
- lance ros_install.sh si ROS n'est pas installé
- installe les dépendances des packages présents dans le workspace de cyclosafe
- build les packages présents dans le workspace de cyclosafe
- configure et activent les services systemd s'ils ne sont pas déjà activés

### ros_install.sh

Basé sur ce repo : https://github.com/Ar-Ray-code/rpi-bullseye-ros2

- Télécharge le paquet de ROS2-jazzy pour debian bookworm et l'installe.
- Rétrograde **empy** à la version *3.3.4* afin d'éviter une erreur de compabilité avec des dépendances de ROS.
- Installe des dépendances manquantes au paquet .deb
- Initialise et mets à jour la base de données rosdep permettant de référencer des dépendances spécifiques à ROS
- Installe un ensemble d'autres dépendances (voir le script pour les détails)

### systemd/setup_services.sh

Teste la présence de chacun des 4 services nécessaires à cyclosafe, et les configure puis les active si nécessaire.

Voir [**la page dédiée aux services**](systemd/README) pour plus d'info.

### ip_config.sh

Créer et active un profil réseau permettant de se connecter au raspberry depuis l'hôte.

Il s'agit d'un condensé de la configuration réseau à effectuer sur le raspberry.

Voir les [**instruction complètes**](../../network.md) pour plus de détails

## cmdline.txt

S'ajoute au fichier de configuration ***/boot/firmware/cmdline.txt*** (ce qui est fait automatiquement avec le script **setup.sh**).

> **ipv6.disable=1** : Désactive ipv6 pour éviter des problèmes DNS lors du téléchargement de certains paquets (notamment en passant par le réseau de l'IGN)

> ***Note*** : si jamais le service systemd gpiod.service détecté perpétuellement un appui sur le bouton au démarrage et que le raspberry devient inaccessible, il est possible de modifier ***/boot/firmware/cmdline.txt*** via un lecteur de carte sd de cette façon là afin de masquer le service au démarrage :
>
	> ipv6.disable=1 systemd.mask=gpiod.service

## config.txt

S'ajoute au fichier de configuration ***/boot/firmware/config.txt*** (ce qui est fait automatiquement avec le script **setup.sh**).

> **dtoverlay=gpio_pin=16,gpio_pull=up** : configure le gpio 16 en pull_up mode

> **dtoverlay=disable-bt** : désactive le bluetooth

> **enable_uart=1** : active l'uart

> **dtoverlay=uartN** : active les interfaces uart utilisées par cyclosafe


## .bashrc

Permet de sourcer dans un terminal l'environnement de cyclosafe, c'est à dire :
- ROS2 et ses utilitaires
- les noeuds ROS2 de cyclosafe
- le fichier **.env** contenant les variables d'environnement propres à cyclosafe
- des alias utiles :
	- **cy_core_build** : compile l'ensemble des packages (en respectant le repertoire d'installation) et source automatiquement les ajouts
		> `alias cy_core_build="cd $CYCLOSAFE_WORKSPACE; colcon build --symlink-install --parallel-workers=2; source ./setup/.bashrc; cd -"`
	- **cy_core_clean** : nettoie le workspace ROS de cyclosafe en repartant sur une installation vierge
		> `alias cy_core_clean="cd $CYCLOSAFE_WORKSPACE; rm -rf build/ install/ log/; source ./setup/.bashrc; cd -;"`

**Usage**:
`source ./.bashrc`

**Pour sourcer automatiquement l'environnement à l'ouverture d'un terminal** :
`$(cat ./bashrc) >> ~/.bashrc`

## .env

Définit l'ensemble des variables d'environnement utilisées par cyclosafe.

> **CYCLOSAFE_WORKSPACE** : répertoire où se trouve les dossiers principaux de cyclosafe (cad le sous-dossier **core** du repo)

> **CYCLOSAFE_RECORD** : répertoire où sauvegarder les enregistrement de données

> **CYCLOSAFE_LOGS** : répertoire où sauvegarder les logs des différents scripts ([**gps_time.sh**](../scripts/README#gps_timesh) et [**gpio.sh**](../scripts/README#gpiosh))

> **GPS_SERIAL_PORT** : serial device sur lequel est branché le GPS. Utilisé par [**gps_time.sh**](../scripts/README#gps_timesh) pour mettre à jour l'heure et la date du système.

> **SCRIPTS_PATH** : répertoire où trouver les différents scripts lancés par les services systemd. A priori équivalent à **$CYCLOSAFE_WORKSPACE/scripts**

> **SHUTDOWN_DELAY** : temps imparti à l'arrêt du service [**cyclosafed.service**](systemd/README#cyclosafedservice) pour se terminer correctement avant d'éteindre le raspberry.