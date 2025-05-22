# cyclosafe_config

Package python dont la seule fonction est d'exporter les classes `Sensor` et `SensorTypeEnum`, utiles à l'intégration d'un node dans une launch description, en permettant d'uniformiser la déclaration des paramètres à travers plusieurs launch description.

Ces classes sont également utilisées par l'environnement [**viewer/**](../../../viewer/README.md), raison pour laquelle elles sont isolées dans un package.


## Classe Sensor

Classe helper qui a pour but d'uniformiser la déclaration d'un capteur (et de ses différentes propriétés) à travers différentes launch descriptions (principalement celle du package cyclosafe et celle du viewer).

Essentiellement elle permet de wrapper l'instanciation d'un **Node()** dans les launch description en automatisant la résolution des ports et des couleurs associées aux capteurs pour leur future visualisation dans Rviz.

Cette classe étant aussi utilisée par [**viewer/**](../../../viewer/README.md), elle permet de faire correspondre à une certaine config d'enregistrement la visualisation qui lui est associée.

Voir [**config.py**](../cyclosafe/launch/config.py) pour un exemple d'utilisation.

Les paramètres sont donnés sous forme de *kwargs* lors de l'instanciation.

## Paramètres

> **package** : string
> - nom du package auquel appartient le noeud
> - **requis**

> **executable** : string
> - nom de l'executablee du noeud
> - **requis**

> **namespace** : string
> - namespace utilisé par le noeud
> - **requis**, peut être vide

> **port** : string
> - nom du package auquel appartient le noeud
> - **requis**, peut être **None** ou vide
> - Si **None**: **port_hint** sera utilisé pour essayer de résoudre le port définitif

> **type** : SensorTypeEnum
> - type de capteur
> - **requis**
> - Valeurs possibles : 
> 	- SensorTypeEnum.RangeSensor (=0)
> 	- SensorTypeEnum.Lidar360Sensor (=1)
> 	- SensorTypeEnum.GPSSensor (=2)
> 	- SensorTypeEnum.CameraSensor (=3)

> **enable** : bool
> - défaut: **True**
> - permet de prendre en compte ou non le capteur dans la config

> **parameters** : [dict[str, Any]]
> - défaut: **""**
> - contient une list d'un seul dictionnaire de paramètres qui seront transmis à ROS.
> - si le **port** est dynamiquement résolu grâce à **port_hint**, les paramètres ayant pour nom **port**, **serial_port** ou **port_name** seront mis à jour.
> - défaut: **[]**

> **description** : str
> - purement indicatif

> **transform** : [str]
> - défaut: **None**
> - lorsque défini, correspond à la liste des arguments qui seront transmis à l'execution d'un **static_transform** lors de la visualisation
> **Obsolete** : préférer l'utilisation d'un modèle URDF pour la visualisation.
> - <ins>**Exemple**</ins> :
> 	- ["--x", "5.0", "--y", "0.0", "--z", "0.0", "--roll", "-1.57", "--pitch", "1.57", "--yaw", "0", "--frame-id", "world", "--child-frame-id", "my_sensor_frame"],
> 	- Transpose la frame (*référentiel*) du capteur dans la frame world

> **description** : str
> - défaut: **""**
> - purement indicatif

> **topic** : str
> - défaut: **"range"**
> - topic sur lequel seront publiés les données
> - surtout utilisé par les outils de visualisation afin d'identifier les capteurs dans les rosbag

> **color** : str | std_msgs.msg.ColorRGBA
> - défaut : **None**
> - si **"auto"**, la couleur est automatiquement attribuée en fonction de l'ordre de déclaration du capteur
> - accepte les objets ColorRGBA ou des valeurs littérales comme **red, green, magenta, ...**
> - sera utilisé pour colorier certaines données dans les outils de visualisation.

> **port_hint** : str
> - défaut: **None**
> - A utiliser en conjonction avec **port = None**
> - Contient une partie de l'id vendeur d'une interface sériale (trouvable dans */dev/serial/by-id/...*) à partir duquel le port final va tenter d'être résolu.
> - <ins>**Exemple**</ins> :
> 	- le pattern **"if00-port0"** peut renvoyer **/dev/serial/by-id/e40dc7ec375aee118e528bdc8ffcc75d-if00-port0**
> 	- il s'agit symlink qui point vers **../../ttyUSB1**
> 	- le port définitif sera donc : **/dev/serial/by-id/../../ttyUSB1**

> **delay** : float
> - défaut: **None**
> - **unité** : secondes
> - indique un délai au lancement du noeud
> - peut-être utiliser pour inciter les noeuds à se lancer dans un certain ordre
> - **Attention** : il n'y aucune garantie que cet ordre sera respecté

> **log_level** : str
> - défaut: **"info""**
> - permet d'indiquer pour un noeud un niveau de log potentiellement différent du reste de la config 
> - prend la priorité sur l'argument **log_level** de la launch description
