# CYCLOSAFE

> The aim of the CycloSafe project is to rigorously quantify all the dangers to which cyclists are exposed during their journeys, so that we can better target concrete actions to improve their safety.
> 
> To do this, we have equipped a bicycle with LIDARs (Light Detection and Ranging), a camera and a GNSS antenna.

Ce repo contient le firmware destiné à tourner sur un **raspberry** pour faire fonctionner l'ensemble des capteurs et **prendre les mesures**, les différents outils pour **exporter**, **visualiser** et **analyser** les données prises et les différents fichiers permettant l'**impression 3D du boitier** et la fabrication du **PCB dédié**.

https://github.com/user-attachments/assets/d5b68dda-f3fe-4f50-86e3-c243446fb8e7

## Structure

> [**core/**](core/README.md) : répertoire destiné à être installé sur le raspberry. Contient :
> 	- script d'installation de ROS
> 	- script de mise en place de l'environnement cyclosafe
> 	- script de mise en place des services systemd
> 	- code source des différents noeuds ROS

> [**design/**](design/README.md) : contient les fichiers relatifs à l'usinage du boitier et du PCB dédié.

> [**scripts/**](scripts/README.md) : contient des scripts utiles à la récupération et l'export des données

> [**viewer/**](viewer/README.md) : répertoire destiné à être installé sur l'hôte. Contient un ensemble d'outils pour visualiser et analyser les données..

> [**network.md**](network.md) : instructions pour la configuration réseau sur l'hôte et sur le raspberry permettant l'interaction entre les deux machines

