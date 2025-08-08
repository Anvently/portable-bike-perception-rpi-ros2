# CYCLOSAFE

> The aim of the CycloSafe project is to improve cyclist safety by rigorously quantifying the risks they face, with a particular focus on those occurring when being overtaken by a motor vehicle.
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
> - [**ROS2.md**](viewer/ROS2.md) : documentation liée à ROS2 et son usage dans le projet.

> [**network.md**](network.md) : instructions pour la configuration réseau sur l'hôte et sur le raspberry permettant l'interaction entre les deux machines


## About this project

This project was carried out as part of an internship at the **Institut national de l’information géographique et forestière (IGN)**, within a broader research initiative aiming **to quantify the risks faced by cyclists when being overtaken by motor vehicles**.

This repository focuses specifically on the **design and implementation of the acquisition module** used to collect measurement data for this study, as well as providing a set of **tools for processing, visualizing, and analyzing** the recorded data.

It does not present or discuss the **results** of the study itself, and should not be directly associated with the conclusions or interpretations of that research.

For more information on the study, see:
Emmanuel Cledat, Dirk Lauinger, Aymeric Dutremble, Maeve Blarel, Damien Louis Peller, Tristan Geslain, Alexandre Esteoulle, Elisabeth Giroux, Gabin Bourlon, Nicolas Pirard, Eric Ta — ***Cyclo-Safe: Quantitative study of the risks to which cyclists are exposed during their daily commute***.

## License

This project is licensed under the **CeCILL-B v1** license, a free software license fully compliant with French law, **based on the BSD 2-Clause license** and the principles of open source.

**Scope of the license** — The following materials are covered by the CeCILL-B license:
- Firmware code based on ROS2
- All project installation scripts and data analysis scripts
- Source code of data analysis tools, based on Qt and ROS2
- PCB design files
- 3D models and any corresponding f3d source files
- Assembly and user manuals for the module

You are **free to**:
- Use, modify, and redistribute this project under the terms of the CeCILL-B license
- Use this project for both **academic** and **commercial** purposes, without the strong copyleft constraints of GPL-like licenses
- Incorporate this code into proprietary software, as long as you respect the attribution requirements

You must:
- Include a copy of the CeCILL-B license with any redistributed version
- Retain notices of authorship and copyright
- Clearly indicate any modifications made

Author: [Nicolas Pirard (@Anvently)](https://github.com/Anvently)  
Contact: [pirard.nicolas@hotmail.fr](mailto:pirard.nicolas@hotmail.fr)
Additional contributors: See [CONTRIBUTORS.md](./CONTRIBUTORS.md)

📄 Full license text: [CeCILL-B v1](http://www.cecill.info/licences/Licence_CeCILL-B_V1-en.html)  

📄 Version française: [CeCILL-B v1](https://cecill.info/licences/Licence_CeCILL-B_V1-fr.html)  

---
