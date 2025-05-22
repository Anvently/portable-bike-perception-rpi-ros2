# ROS2

Le projet cyclosafe repose sur **ROS2** pour la prise et l'enregistrement des mesures.

> ***ROS (Robot Operating System)*** provides libraries and tools to help software developers create robot applications.
> 
> It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.
> 
> ROS is licensed under an open source, BSD license.

Source: https://wiki.ros.org/ 

> Pour résumer ROS2 est une surcouche logicielle permettant de formaliser les échanges entre processeurs.
> 
> Des outils et des bibliothèques sont fournis afin de faciliter le développement d'applications robotiques.

Source : https://web.enib.fr/~kerhoas/automatique-robotique/robot-mobile-rpi-ros2/presentation-ros2/


ROS est basé sur une architecture de noeuds exécutables pouvant publier et/ou souscrire à des [topics](#topic).


- [ROS2](#ros2)
- [Pourquoi utiliser ROS 2 dans ce projet ?](#pourquoi-utiliser-ros-2-dans-ce-projet-)
  - [Représentation des données](#représentation-des-données)
  - [Visualisation des données](#visualisation-des-données)
  - [Orchestration](#orchestration)
  - [Sérialisation et enregistrement des données](#sérialisation-et-enregistrement-des-données)
  - [Paramètrisation des tests](#paramètrisation-des-tests)
  - [Modularité](#modularité)
  - [Open source](#open-source)
- [Glossaire ROS2](#glossaire-ros2)
  - [Action](#action)
  - [Bag (ou rosbag)](#bag-ou-rosbag)
  - [Build](#build)
  - [Colcon - *Colcon*](#colcon---colcon)
  - [Discovery](#discovery)
  - [Frame - *Repère*](#frame---repère)
  - [Installation](#installation)
  - [Interface](#interface)
  - [Launch description - *Ou launch file*](#launch-description---ou-launch-file)
  - [Client librairie](#client-librairie)
  - [Message](#message)
  - [Node - *Noeud*](#node---noeud)
  - [Package](#package)
  - [Parameter - *Paramètres*](#parameter---paramètres)
  - [Rclcpp - *ROS Client Library C++*](#rclcpp---ros-client-library-c)
  - [Rclpy - *ROS Client Library Python*](#rclpy---ros-client-library-python)
  - [Rosdep - *ROS Dependency*](#rosdep---ros-dependency)
  - [Rviz - *ROS Visualization*](#rviz---ros-visualization)
  - [Service](#service)
  - [tf2](#tf2)
  - [Topic](#topic)
  - [URDF - *Unified Robot Description Format*](#urdf---unified-robot-description-format)
  - [Workspace - *Environnement de travail*](#workspace---environnement-de-travail)

# Pourquoi utiliser ROS 2 dans ce projet ?

## Représentation des données

Les données manipulées par le projet inclue des données numériques et spatiales de différentes nature, cela nécessite une capacité à représenter et à visualiser ces données.

**ROS2**, notamment grace aux [interfaces](#interface) définies dans `sensor_msgs` permet la représentation :
- de nuages de points via `sensor_msgs/msg/LaserScan`
- de distances via `sensor_msgs/msg/Range`
- d'images compressées via `std_msgs/msg/CompressedImage`
- de coordonnéées GPS via `sensor_msgs/msg/NavSatFix`

Ces [messages](#message) sont compatibles et s'intègrent parfaitement aux outils de visualisation comme [`Rviz`](#rviz---ros-visualization) ou `Mapviz`.

## Visualisation des données

L'usage de capteurs, et notamment de capteurs complexe (comme les les lidars 360°), impose, en dehors de l'analyse, de pouvoir visualiser correctement les données.

Il faut également pouvoir transformer les systèmes de coordonnées vers un système global, les capteurs ayant chacun leur propre référence.

- [`Rviz`](#rviz---ros-visualization), [package](#package) installé avec ROS2 est un excellent outil pour visualiser les données dans un espace 3D.
- [`tf2`](#tf2) et [`urdf`](#urdf---unified-robot-description-format) qui s'intègrent parfaitement à [Rviz](#rviz---ros-visualization), permettent de simplifier et d'automatiser toutes les transformations nécessaires.

## Orchestration

La multitude de capteur à utiliser impose de faire tourner en simultanné plusieurs programmes prenant des mesures. ROS2 permet :
   - **via les [launch description](#launch-description---ou-launch-file)** : gestion du lancement de plusieurs [noeuds](#node---noeud) simultannés avec une possibilité de définir et mutualiser des [paramètres](#parameter---paramètres)
   - via les [**topics**](#topic) et les [**services**](#service) : communication entre plusieurs [noeuds](#node---noeud), permettant une potentielle analyse en directe à partir de données mesurées par des capteurs différents

## Sérialisation et enregistrement des données

Il est nécessaire de sérialiser et d'enregistrer dans un dossier les données prises. Il est donc essentiel d'optimiser l'espace de stockage que celle-ci occuperont :

   - ROS2 fournit avec les [`rosbag`](#bag-ou-rosbag) un outil extremement pour sérialiser et compresser les données sans perte.
  
## Paramètrisation des tests

Le besoin de prototyper et tester rapidement différentes configurations et différents capteurs impose des lancements paramètriques :
   - ROS2 implémente un système de [paramètres](#parameter---paramètres) pouvant être transmis de multiples façons différentes, et notamment modifier au runtime
   - Ce système de [paramètre](#parameter---paramètres) s'intègre parfaitement avec les [**launch description**](#launch-description---ou-launch-file)
  
## Modularité

Les différents [noeud](#node---noeud) et scripts étant codés dans des langages différents (c++, python, bash) et en partie fournis par les fabricants, il est nécessaire de les assembler ensemble et de résoudre un certain nombre de dépendances.

- La structure en [package](#package) de ROS pallie très bien à cette contrainte
- [**rosdep**](#rosdep---ros-dependency) permet de résoudre les dépendances
- [**colcon**](#colcon---colcon) permet d'automatiser et de déléguer le [build](#build) et l'[installation](#installation) des différents [packages](#package)

## Open source

Les outils fournis par ROS sont en développement constant et très bien documenté.

De nombreuses APIs ([rclpy](#rclpy---ros-client-library-python), [rclcpp](#rclcpp---ros-client-library-c), rosbag2_py) sont disponibles pour créer et déployer des outils personnalisés.

[`cyclosafe_player`](src/cyclosafe_player/README.md) utilise notamment `rosbag2_py` pour manipuler la lecture de [rosbag](#bag-ou-rosbag).

Par ailleurs les fabricants de capteurs fournissent généralement des **SDK** (Software Kit Development) pour **ROS2** permettant d'intégrer leur capteur au système ROS.

# Glossaire ROS2

## Action

Non utilisé par Cyclosafe.

Système de communication synchrone entre deux noeuds apparenté au [**service**](#service), permettant une **réponse** à une **requête**, à la différence qu'il permet en plus de fournir un **feedback** en temps réel sur l'avancement de la **requête**.

## Bag (ou rosbag)
Fichier ou ensemble de fichiers contenant l'enregistrement de [**messages**](#message) publiés sur un ou plusieurs [**topics**](#topic). Un [**bag**](#bag-ou-rosbag) peut être rejoué ultérieurement de façon à simuler et à reproduire l'ensemble des [messages](#message) qui ont été publiés au moment de l'enregistrement.

## Build
Processus de compilation et de transformation du code source des [**packages**](#package) en exécutables et bibliothèques utilisables. Cette étape est réalisée par [**colcon**](#colcon---colcon) dans le répertoire `build/` du [**workspace**](#workspace---environnement-de-travail).

## Colcon - *Colcon*
Outil permettant de [build](#build) et d'[installer](#installation) un ou l'ensemble des [**packages**](#package) sur le [**workspace**](#workspace---environnement-de-travail). Il gère les dépendances entre [packages](#package) et automatise le processus de compilation.

## Discovery
Processus automatique par lequel un [noeud](#node---noeud) prend connaissance d'un autre et détermine la façon d'échanger avec lui. Ce mécanisme permet aux [noeuds](#node---noeud) de se connecter dynamiquement sans configuration manuelle préalable.

## Frame - *Repère*

Système de coordonnées 3D de référence (ou repère) dans lequel sont publiés des [messages](#message) contenant des données spatiales. Les [messages](#message) publiés par les [noeuds](#node---noeud) ont une propriété `frame` indiquant à quel [frame](#frame---repère) ils sont référencés.

**Ex**: les [messages](#message) publiés sur la [frame](#frame---repère) `lidar1_frame` contiennent des données spatiales référencées par rapport au `lidar1`.

Les [frames](#frame---repère) et les transformations de l'une à l'autre sont gérées par [`tf2`](#tf2), qui maintient un arbre de transformations entre les différents repères.

## Installation
Processus qui suit le [**build**](#build) et qui consiste à placer les exécutables, bibliothèques et les ressources partagées dans le répertoire `install/` du [**workspace**](#workspace---environnement-de-travail), les rendant ainsi utilisables par l'utilisateur.

## Interface
Métaclasse pouvant définir de nouveaux types de [messages](#message), [services](#service) et actions utilisables dans différents langages de programmation (via les librairies [rclcpp](#rclcpp---ros-client-library-c) et [rclpy](#rclpy---ros-client-library-python)).

## Launch description - *Ou launch file*

Fichier dans lequel est décrit une configuration de lancement simultanné de plusieurs [noeuds](#node---noeud) ROS (ou autres executables).

## Client librairie
Librairie spécifique à un langage de programmation permettant à un [noeud](#node---noeud) de communiquer avec un autre [noeud](#node---noeud) utilisant un autre langage de programmation. Les deux principales étant [**rclpy**](#rclpy---ros-client-library-python) (Python) et [**rclcpp**](#rclcpp---ros-client-library-c) (C++).

## Message
Type de donnée spécifique à ROS échangée entre plusieurs [noeuds](#node---noeud) via un [**topic**](#topic). Les [messages](#message) définissent la structure des données transmises et peuvent contenir des types primitifs ou des structures complexes.

## Node - *Noeud*
Entité utilisant ROS pour communiquer avec d'autres [noeuds](#node---noeud). Un [noeud](#node---noeud) est un processus qui peut publier et/ou souscrire à des [**topics**](#topic), offrir ou utiliser des [services](#service), et participer au système de communication ROS.

## Package
Module de base de ROS2. Un [package](#package) peut contenir des [noeuds](#node---noeud), des [interfaces](#interface) personnalisées, des ressources, des fichiers de configuration, des scripts, des librairires, du code source, etc...

On distingue :
- les **core [package](#package)**, qui constituent les outils du système **ROS2** (trouvables dans `/opt/ros/jazzy/` et sourcés par `/opt/ros/jazzy/setup.bash`).
  - **Ex** : [rviz](#rviz---ros-visualization), [tf2](#tf2), sensor_msgs, [rclcpp](#rclcpp---ros-client-library-c), ...
- les [**packages**](#package) **utilisateurs**, contenus dans le répertoire [`src/`](./src/) du [workspace](#workspace---environnement-de-travail) et qui correspondent aux [packages](#package) installés par l'utilisateur. 
  

## Parameter - *Paramètres*
Variables de configuration des [noeuds](#node---noeud) pouvant être transmis de multiples façons différentes (via terminal ou des [launch description](#launch-description---ou-launch-file)), et notamment modifiées au runtime (via des [services](#service)).

## Rclcpp - *ROS Client Library C++*
[**Librairie client**](#librairie-clien) pour le langage C++ permettant de développer des [noeuds](#node---noeud) ROS. Elle fournit les API nécessaires pour créer des [noeuds](#node---noeud), publier/souscrire à des [topics](#topic), et utiliser les [services](#service) ROS.

## Rclpy - *ROS Client Library Python*
[**Librairie client**](#librairie-clien) pour le langage Python permettant de développer des [noeuds](#node---noeud) ROS. Elle offre une [interface](#interface) Python pour toutes les fonctionnalités de base de ROS2.

## Rosdep - *ROS Dependency*
Outil permettant de résoudre et d'automatiser l'installation des dépendances des [**packages**](#package). Il analyse les fichiers de configuration des [packages](#package) et installe automatiquement les dépendances système requises.

## Rviz - *ROS Visualization*
Outil de visualisation 3D pour ROS permettant d'afficher des données de capteurs, des modèles de robots, des cartes, des trajectoires et d'autres informations spatiales.

C'est sur cet outil que repose la visualisation des données dans l'espace via **viewer.launch.py**.

## Service

Système de communication synchrone entre deux [noeuds](#node---noeud), offrant la possibilité de réponse à une demande.

<ins>**Ex**</ins> : le [**noeud**](#node---noeud) **caméra** de **cyclosafe_core** offre un [service](#service) ```cyclosafe_interfaces/srv/SaveImages``` permettant de demander l'enregistrement dans des fichiers des dernières images prises.

## tf2

Système de transformation de coordonnées entre les différentes [`frames`](#frame---repère).

Le [package](#package) `tf2_ros` permet notamment de diagnostiquer l'ensemble des [frames](#frame---repère) et de publier des transformations statiques via `static_transform_publisher` (c'est à dire une transformation constante d'une [frame](#frame---repère) parente vers une [frame](#frame---repère) enfant).

## Topic
Thème sur lequel sont publiés des [**messages**](#message) et auquel un [**noeud**](#node---noeud) peut souscrire. Les [topics](#topic) permettent une communication asynchrone de type publish/subscribe entre les [noeuds](#node---noeud).

## URDF - *Unified Robot Description Format*
Format XML standardisé pour décrire la structure physique d'un robot, incluant ses liens (links), articulations (joints), propriétés visuelles et de collision.

**viewer.launch.py** utilise un modèle [URDF](#urdf---unified-robot-description-format) pour positionner dans l'espace des modèles 3D du vélo et des capteurs, et pour définir les transformations à effectuer pour replacer les données dans l'espace.

Fonctionne avec le [package](#package) `robot_state_publisher` qui prend en entrée une description [URDF](#urdf---unified-robot-description-format) et génère automatiquement l'arbre de transformation [`tf2`](#tf2) associé.

## Workspace - *Environnement de travail*
L'environnement de travail contenant l'ensemble du code source des [**packages**](#package) et les répertoires dans lesquels ils sont [build](#build) et [installés](#installation).

Il est nécessaire de sourcer le [**workspace**](#workspace---environnement-de-travail) afin d'utiliser les [packages](#package) qui y sont installés.

On distingue :
- le [**workspace**](#workspace---environnement-de-travail) principal où ROS2 et l'ensemble des [**packages**](#package) core sont installés (ex: `/opt/ros/jazzy` pour Jazzy)
- le [**workspace**](#workspace---environnement-de-travail) utilisateur dans lequel on peut installer ses propres [packages](#package) (ex: `~/cyclosafe/viewer`).

Le [**workspace**](#workspace---environnement-de-travail) utilisateur contient généralement 4 dossiers :
- **src/** : contient le code source des [packages](#package) à installer
- **build/** : répertoire dans lequel les [packages](#package) sont [buildés](#build) par [**colcon**](#colcon---colcon)
- **install/** : répertoire dans lequel les [packages](#package) sont [installés](#installation) par [**colcon**](#colcon---colcon) et où se trouvent les éventuelles ressources partagées
- **log/** : contient toutes les logs concernant le [build](#build) et l'[installation](#installation)

