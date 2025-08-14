# Cyclosafe viewer

Ce dossier contient l'ensemble des packages et utilitaires permettant de visualiser les données du projet cyclosafe.

- [Cyclosafe viewer](#cyclosafe-viewer)
  - [Structure](#structure)
  - [Installation](#installation)
    - [ROS2](#ros2)
    - [Installation de ROS2](#installation-de-ros2)
  - [Usage](#usage)
    - [Sourcer le workspace](#sourcer-le-workspace)
    - [Résoudre les dépendances avec rosdep](#résoudre-les-dépendances-avec-rosdep)
    - [Build et installation des packages](#build-et-installation-des-packages)
    - [Alias fournis par le script .bashrc](#alias-fournis-par-le-script-bashrc)
      - [cy\_viewer\_build](#cy_viewer_build)
      - [cy\_viewer\_clean](#cy_viewer_clean)
    - [Visualiser les données à partir d'un bag](#visualiser-les-données-à-partir-dun-bag)
    - [Visualiser les données en direct](#visualiser-les-données-en-direct)
  - [Activer mapviz et rviz\_satellite (facultatif)](#activer-mapviz-et-rviz_satellite-facultatif)

## Structure

> [**./**](core/README.md) : répertoire courant, correspond aussi au workspace  utilisé par **ROS2** dans lequel les packages sont installés.
> 
> [**src/**](core/README.md) : répertoire contenant le code source des outils de visualisation et d'analyse, c'est àdire:
>   - [**cyclosafe_config**](../core/src/cyclosafe_config/README.md) : symlink vers le package python cyclosafe_config du [**core/**](../core/), nécessaire  sur l'hôte car utilisé par `cyclosafe_viewer`.
>   - [**cyclosafe_interfaces**](../core/src/cyclosafe_interfaces/README.md) : symlink vers le package python cyclosafe_interfaces du [**core/**](../core/), nécessaire  sur l'hôte car utilisé par le script [`gpx_exporter`](../scripts/README.md#gpx_exporterpy).
>   - [**cyclosafe_player**](src/cyclosafe_player/README.md) : package python permettant de jouer et de contrôler la lecture de rosbag et de visualiser graphiquement des distances
>   - [**cyclosafe_viewer**](src/cyclosafe_viewer/README.md) : package python permettant de visualiser les données sur Rviz
>   - [**mapviz (submodule)**](src/mapviz/README.md) : package extérieur permettant de visualiser des données ROS2 (comme RViz) sur des cartes et de les intégrer à des outils cartographiques.
>   - [**rviz_satellite (submodule**](src/rviz_satellite/README.md) : package extérieur permettant d'intégrer des tuiles cartographiques sur Rviz à partir de coordonnées GPS.
>
> [**setup/**](setup/) : contient des scripts permettant de [**sourcer plus facilement**](#sourcer-le-workspace) l'environnement de travail et de [**déclarer des alias**](#alias-fournis-par-le-script-bashrc)

## Installation

### ROS2

Le projet cyclosafe repose sur **ROS2** pour la prise et l'enregistrement des mesures.

> ***ROS (Robot Operating System)*** provides libraries and tools to help software developers create robot applications.
> 
> It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.
> 
> ROS is licensed under an open source, BSD license.

Source: https://wiki.ros.org/ 

Pour une introduction aux différents concepts de ROS et aux raisons pour lesquelles il est utilisé dans ce projet, voir : [**introduction à ROS**](ROS2.md).

### Installation de ROS2

> <ins>**Important</ins>** :
> 
> Les packages et outils de Cyclosafe ont été développés sur la distribution **Jazzy** de ROS2, qui la distribution stable la plus récente de ROS2.
> 
> Il est donc essentiel d'installer **ROS2 Jazzy** ou la distribution en développement (**ROS2 Rolling**).
>
> Il n'y a pas de garantie que l'ensemble des outils d'analyses fonctionneront sur les distributions antérieures.

Ce qui est suit est un résumé des instructions d'installation fournit par [**la documentation officielle de Jazzy**](https://docs.ros.org/en/jazzy/Installation.html).

Ajout de la clé GPG de ROS2:

~~~
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
~~~

Ajout du repo à la liste des sources:

~~~
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
~~~


Installation des outils de développement:

~~~
sudo apt update && sudo apt install ros-dev-tools
~~~

Installation de la suite desktop de ROS2 (incluant les outils de visualisation) :

~~~
sudo apt install ros-jazzy-desktop
~~~

On peut tester l'installation en lancant dans un terminal :

~~~
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
~~~

Et dans un autre terminal :

~~~
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_py listener
~~~

## Usage

Le dossier courant [**viewer/**](.) est l'environnement de travail de ROS2 (cad. le workspace).

### Sourcer le workspace

A chaque fois qu'on ouvre un nouveau terminal, il est nécessaire de sourcer cet environnement afin d'avoir accès à **ROS2** ainsi qu'aux outils propres à Cyclosafe.

Cela se fait normalement en sourcant le dossier `install` du workspace.

Cependant lors de la première installation, le build et l'installation des packages n'a pas encore été effectuée donc le dossier `install` n'existe pas encore.

Afin de simplifier ces opérations, il est fourni un script [**.bashrc**](./setup/.bashrc).

> Ce script permet :
> - de sourcer l'environnement de travail, ou à défaut d'installation l'environnement par défaut de **ROS2** (*/opt/ros/jazzy/setup.bash*)
> - d'exporter des variables d'environnements propres au projet
> - déclare des alias facilitant le build et l'installation des packages

Afin d'utiliser les scripts fournis par cyclosafe et les variables d'environnement du projet, il est nécessaire de créer un fichier `./setup/.env`.

Depuis le repertoire courant `viewer/` vous pouvez simplement exécuter ces commandes :

~~~
export CYCLOSAFE_WORKSPACE=$PWD
eval "echo \"$(cat ./setup/.env.template)\"" > ./setup/.env
~~~

Depuis le répertoire courant (`viewer/`), vous pourrez ensuite sourcer le script via :
~~~
source ./setup/.bashrc
~~~

> Afin de ne pas avoir à répéter cette opération pour chaque nouveau terminal ouvert, vous pouvez l'ajouter au **.bashrc**  de votre dossier utilisateur (**~**) :
> ~~~
> echo source $CYCLOSAFE_WORKSPACE/setup/.bashrc >> ~/.bashrc
> ~~~
> Le script sera désormais automatiquement executé à l'ouverture d'un terminal.


### Résoudre les dépendances avec rosdep

Chaque package définit dans un fichier **package.xml** les dépendances dont il a besoin pour fonctionner.

ROS2 dispose de l'outil **rosdep** pour lire automatiquement ces fichiers et installer les dépendances manquantes.

Lors de la première utilisation de **rosdep**, il vous sera demandé d'initialiser et de mettre à jour les sources avec lesquelles rosdep travaille (sa base donnée).

~~~
sudo rosdep init
rosdep update
~~~

> **Important** : ne pas utiliser `sudo` avec rosdep update car ce n'est pas nécessaire et peut causer des problèmes de permissions par la suite.

Depuis la racine de l'environnement, vous pouvez ensuite installer les dépendances :

~~~
cd  $CYCLOSAFE_WORKSPACE
rosdep install -ry --from-path src/
~~~

> `--from-path` précise le dossier à partir duquel seront cherchés récursivement tous les packages dont il faut résoudre les dépendances
> 
> `-r` demande à l'installation de se poursuivre en cas d'échec à l'installation d'un package ou d'une dépendance introuvable
>
> `-y` évite d'avoir à confirmer l'installation des packages lorsque demandé dans le terminal

Pour simplement vérifier les dépendances manquantes et si celles-ci peuvent être résolues vous pouvez utiliser à la place la commande :
~~~
rosdep check --from-path src/
~~~

### Build et installation des packages

Dans le cas d'une nouvelle installation, le dossier **install/** contenant les exécutables des noeuds et les éventuelles ressources partagées n'existe pas encore.

Il est nécessaire de **build** le code source (c'est à dire compiler le code c++ et éxecuter d'éventuels scripts ) et d'**installer** les packages (c'est à dire copier tous les exécutables produits par le build et les ressources partagées dans le dossier **install/**).

Ces étapes doivent également être faites lors de modifications du code source (pour les packages python cela n'est pas nécessaire à condition d'utiliser l'option **symlink-install** lors du build).

**ROS2** dispose pour cela de l'outil **colcon** qui permet de build l'ensemble du workspace (qu'ils soient en python ou en c++) tout en résolvant les dépendances des packages et l'ordre dans lequel ils doivent être build.

> <ins>Important</ins> : Il est essentiel d'exécuter la commande de build depuis la racine du **workspace**, car les dossiers **install/** et **build/** sont créés dans le répertoire courant.
>
> Sinon le workspace sera scindé en plusieurs répertoires d'installation et le script d'initialisation ne sourcera pas le **workspace** le plus récent.

Depuis le répertoire racine du workspace (*ex:* **~/cyclosafe/viewer**), exécutez la commande suivante :
~~~
cd $CYCLOSAFE_WORKSPACE
colcon build --symlink-install
~~~

> L'option `symlink-install` est essentielle car lorsque c'est possible, elle ne copie pas dans le dossier `install/` les scripts mais y installe des liens symboliques pointant directement vers le code source.
> 
> Cela permet notamment pour les packages python que les modifications du code source soient prises en compte sans avoir à rebuild le workspace.

> Quelques autres options utiles : 
> - **--symlink-install** : installe des liens symboliques plutôt que des copies lorsque cela est possible
> - **--packages-select pkg1 pkg2** : permet de build et d'installer uniquement des packages spécifiques.
> - **--parallel-workers=N** : définit le nombre maximum de packages qui sont autorisés à être build en simultanné.

### Alias fournis par le script .bashrc

Afin de faciliter la routine de build et d'installation, le script **.bashrc** déclare deux alias dédiés :

#### cy_viewer_build

~~~
$ type cy_viewer_build 
cy_viewer_build is aliased to `cd /home/user/cyclosafe/viewer/; colcon build --symlink-install; source ./setup/.bashrc; cd -'
~~~
Permet :
- `cd $CYCLOSAFE_WORKSPACE` : se déplacer directement à la racine du workspace cyclosafe
- `colcon build --symlink-install` : build les packages en y installant des symlinks
- `cd -` : retourne dans le répertoire précédent

#### cy_viewer_clean

~~~
$ type cy_viewer_clean
cy_viewer_clean is aliased to `cd /home/user/cyclosafe/viewer/; rm -rf build/ install/ log/; source ./setup/.bashrc; cd -'
~~~
Permet de nettoyer le workspace :
- `cd $CYCLOSAFE_WORKSPACE` : se déplacer directement à la racine du workspace cyclosafe
- `rm -rf build/ install/ log/` : supprime les dossiers générés par le build
- `source ./setup/.bashrc` : réinitialise l'environnement à celui de **ROS2 core**
- `cd -` : retourne dans le répertoire précédent

### Visualiser les données à partir d'un bag

Suppose que les enregistrements ont déjà été importés. Vous pouvez pour cela vous référer [**au script d'import**](../scripts/README.md#import_recordingspy).

Pour lancer la visualisation (après avoir sourcé l'environnement) :
~~~
ros2 launch cyclosafe_viewer viewer.launch.py bag:=path_to_mcap_file
~~~
> <ins>**bag**</ins> : Chemin vers le bag converti par le script import_recording.py.
> 
> **Ex:** ~/import/20250521-091239/out/_0.mcap
>
> **Note** : Les fichiers contenus dans le dossier **bag/** (au format .mcap ou .zstd) peuvent aussi être donné en paramètre mais ils ne contiendront qu'une partie de l'enregistrement.

Cette launch description ouvre une fenêtre `Rviz` et une autre fenêtre `cyclosafe_player` permettant de contrôler la lecture des données.

### Visualiser les données en direct

Suppose que le raspberry est allumé et connecté via un cable ethernet.

Pour lancer la visualisation (après avoir sourcé l'environnement) :
~~~
ros2 launch cyclosafe_viewer viewer.launch.py
~~~

Cette launch description ouvre une fenêtre `Rviz`.

> Aucune configuration réseau particulière n'est nécessaire. ROS2 publie par défaut ses messages sur le réseau.


## Activer mapviz et rviz_satellite (facultatif)

Ces deux packages sont des submodule git et ne sont pas donc pas clonés avec le reste du repo.

Tant que l'option `map` de [**viewer.launch.py**](src/cyclosafe_viewer/README.md#viewerlaunchpy) n'est pas utilisée, leur absence ne posera aucun problème.

Pour les initialiser et les cloner :
~~~
git submodule init ./src/mapviz ./src/rviz_satellite
git submodule update ./src/mapviz ./src/rviz_satellite
~~~
