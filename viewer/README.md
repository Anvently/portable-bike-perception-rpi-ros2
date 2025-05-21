# Cyclosafe viewer

Ce dossier contient l'ensemble des packages et utilitaires permettant de visualiser les données du projet cyclosafe.

## Usage

## ROS2

Le projet cyclosafe repose sur **ROS2** pour la prise et l'enregistrement des mesures.

> ***ROS (Robot Operating System)*** provides libraries and tools to help software developers create robot applications.
> 
> It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.
> 
> ROS is licensed under an open source, BSD license.

Source: https://wiki.ros.org/ 

> Pour résumer ROS2 est une surcouche logicielle permettant de formaliser les échanges entre processeurs.
> 
> Des outils et des bibliothèques sont fournis afin de faciliter le développement d’applications robotiques.

Source : https://web.enib.fr/~kerhoas/automatique-robotique/robot-mobile-rpi-ros2/presentation-ros2/


ROS est basé sur une architecture de noeuds exécutables pouvant publier et/ou souscrire à des topics.

### Concepts et outils essentiels

> **Noeud** : entité utilisant ROS pour communiquer avec d'autres noeuds

> **Message** : type de donnée spécifique à ROS échangée entre plusieurs noeuds via un **topic**

> **Topic** : thème sur lequel sont publiés des  **messages** et auquel un **noeud** peut souscrire

> **Découverte** : processus automatique par lequel un noeud prend connaissance d'un autre et détermine la façon d'échanger avec lui

> **Librairie client** : librairie spécifique à un langage de programmation permettant à un noeud de communiquer avec un autre noeud utilisant un autre langage de programmation. Les deux principales étant **rclpy** (python) et **rclcpp** (c++).

> **Package** :

> **Workspace** : l'environnement de travail contenant l'ensemble du code source **packages** et les répertoires dans lesquels ils sont build et installés.
> 
> Il est nécessaire de sourcer le **workspace** afin d'utiliser les packages qui y sont installés.
>
> On distingue le **workspace** principal où ROS2 et l'ensemble des **core** **packages** sont installés (ex: ***/opt/ros/jazzy*** pour jazzy) du **workspace** utilisateur dans lequel on peut installer ses propres packages (ex: **~/cyclosafe/viewer**). 
>
> Le **workspace** utilisateur contient généralement 4 dossiers :
> - **src/** : contient le code source des packages à installer
> - **build/** : répertoire dans lequel les packages sont build par **colcon**
> - **install/** : répertoire dans lequel les packages sont installés par **colcon** et où se trouvent les éventuelles ressources partagées
> - **log/** : contient toutes les logs concernant le build et l'installation

> **rclpy** : **librairie client** pour le langage python permettant de développer des noeuds ROS

> **rclcpp** :  **librairie client** pour le langage c++ permettant de développer des noeuds ROS

> **bag (ou rosbag)** : fichier ou ensemble de fichiers contenant l'enregistrement de **messages** publiés sur un ou plusieurs **topics**. Un **bag** peut-être rejoué ultérieurement de façon à simuler et à reproduire l'ensemble des messages qui ont été publiés au moment de l'enregistrement.

> **rosdep** : outil permettant de résoudre et d'automatiser l'installation des dépendances des **packages**

> **colcon** : outil permettant de build et d'installer un ou l'ensemble des **packages** sur le **workspace**

> 

### Pourquoi utiliser ROS 2 dans ce projet ?

<ins>**Orchestration**</ins> :

La multitude de capteur à utiliser impose de faire tourner en simultanné plusieurs programmes prenant des mesures. ROS2 permet :
   - **via les launch description** : gestion du lancement de plusieurs noeuds simultannés avec une possibilité de définir et mutualiser des paramètres
   - via les **topics** et les **services** : communication entre plusieurs noeuds, permettant une potentielle analyse en directe à partir de données mesurées par des capteurs différents

<ins>**Sérialisation et enregistrement des données**</ins> :

Il est nécessaire de sérialiser et d'enregistrer dans un dossier les données prises. Il est donc essentiel d'optimiser l'espace de stockage que celle-ci occuperont :

   - ROS2 fournit avec les rosbag un outil extremement pour sérialiser et compresser les données sans perte.
  
<ins>**Paramètrisation des tests**</ins> :

Le besoin de prototyper et tester rapidement différentes configurations et différents capteurs impose des lancements paramètriques :
   - ROS2 implémente un système de paramètres pouvant être transmis de multiples façons différentes, et notamment modifier au runtime
   - Ce système de paramètre s'intègre parfaitement avec les **launch description**
  
<ins>**Modularité**</ins> :

Les différents noeud et scripts étant codés dans des langages différents (c++, python, bash) et en partie fournis par les fabricants, il est nécessaire de les assembler ensemble et de résoudre un certain nombre de dépendances.

- La structure en package de ROS pallie très bien à cette contrainte
- **rosdep** permet de résoudre les dépendances
- **colcon** permet d'automatiser et de déléguer le build et l'installation des différents packages

<ins>**Open source**</ins>

Isolation du code des différents capteurs, compilation et installation dans des langages différents (c++, python, scripts bash) :
   - La structure en package de ROS2 et ses outils de build (colcon) sont particulièrement adaptés.

WORK IN PROGRESS
