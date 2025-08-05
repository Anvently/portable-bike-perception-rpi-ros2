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
> - [**ROS2.md**](viewer/ROS2.md) : documentation liée à ROS2 et son usage dans le projet.

> [**network.md**](network.md) : instructions pour la configuration réseau sur l'hôte et sur le raspberry permettant l'interaction entre les deux machines


## License

This project is licensed under the **CeCILL v2.1** license, a free software license fully compliant with French law, **based on the GNU GPL** and the principles of open source and copyleft.

You are **free to**:
- Use, modify, and redistribute this project under the terms of the CeCILL v2.1 license
- Use this project for both **academic** and **commercial** purposes, under the conditions of the CeCILL license
- Access the source code and modify it, provided that any redistribution complies with the CeCILL terms

You must:
- Include a copy of the CeCILL license with any redistributed version
- Retain notices of authorship and copyright
- Clearly indicate any modifications made

Author: [Nicolas Pirard aka Anvently](https://github.com/Anvently)  
Contact: [pirard.nicolas@hotmail.fr](mailto:pirard.nicolas@hotmail.fr)

📄 Full license text: [CeCILL v2.1](http://www.cecill.info/licences/Licence_CeCILL_V2.1-en.html)

📄 Version  française: [CeCILL v2.1](https://cecill.info/licences/Licence_CeCILL_V2.1-fr.html)

---

## Commercial Use

Although the CeCILL license allows commercial use under its terms, a **custom commercial license** may be granted with support or warranty.

If you are interested in using this project (or part of it) in a commercial product with additional guarantees, please contact:

[pirard.nicolas@hotmail.fr](mailto:pirard.nicolas@hotmail.fr)

