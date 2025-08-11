# Cyclo-safe : a multi-sensor bike perception module (Camera, Dual 360° Lidar, GPS, Raspberry Pi, ROS2)

> The aim of the CycloSafe project is to improve cyclist safety by rigorously quantifying the risks they face, with a particular focus on those occurring when being overtaken by a motor vehicle.
> 
> To do this, we have equipped a bicycle with LIDARs (Light Detection and Ranging), a camera and a GNSS antenna.

This repository contains :
- the firmware intended to run on a **Raspberry Pi** to operate all the sensors and **take measurements**
- Various tools to **export**, **visualize**, and **analyze** the collected data
- Files required for **3D printing the case**
- File needed to manufacture the **dedicated PCB**.  
It was developed as part of an experimental data acquisition system for an academic research study on cyclist safety.

https://github.com/user-attachments/assets/d5b68dda-f3fe-4f50-86e3-c243446fb8e7

## Structure

> [**core/**](core/README.md): Directory intended to be installed on the Raspberry Pi. Contains:
> 	- ROS installation script
> 	- CycloSafe environment setup script
> 	- systemd services setup script
> 	- source code for the different ROS nodes

> [**design/**](design/README.md): Contains files related to the machining of the case and the dedicated PCB.

> [**scripts/**](scripts/README.md): Contains scripts useful for retrieving and exporting data.

> [**viewer/**](viewer/README.md): Directory intended to be installed on the host machine. Contains a set of tools for visualizing and analyzing the data.
> - [**ROS2.md**](viewer/ROS2.md): Documentation related to ROS2 and its use in the project.

> [**network.md**](network.md): Instructions for network configuration on both the host and the Raspberry Pi, enabling interaction between the two machines.

## About the Cyclo-safe project

This project is part of the broader **CycloSafe** research initiative conducted at the **Institut national de l’information géographique et forestière (IGN)**, which aims **to quantify the risks faced by cyclists when being overtaken by motor vehicles**.

This repository is dedicated **exclusively to the design and implementation of the acquisition module** used to collect measurement data for the study, along with a set of **tools for processing, visualizing, and analyzing** the recorded data.

It does **not** present or discuss the **results** of the study itself, and should not be directly associated with its conclusions or interpretations.

For more information on the full **CycloSafe** study, see:  
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

📄 French version: [CeCILL-B v1](https://cecill.info/licences/Licence_CeCILL-B_V1-fr.html)  

---
