# Services systemd

Ce dossier contient des templates de configuration pour les différents services utilisés par cyclosafe.

**Important**: pour une installation vierge, préférez l'utilisation du script setup_services.sh qui ajuste plus finement les paramètres en fonction des varaibles d'environnement configurées. Les fichiers de configuration seront automatiquement créés et installés.

Ces services utilisent à chaque démarrage le fichier d'environnement .env du workspace de cyclosafe.

> Une fois configurés, chacun peut-être personnalisé au chemin suivant: ***/etc/systemd/system/[nom_du_service.service]***
> 
> Pour prendre en compte une modification:
> 
> sudo systemctl daemon-reload
>
> sudo systemctl restart [nom_du_service.service]

## cyclosafed.service

C'est le service responsable de lancer la launch description principale de cyclosafe. Voir [**cyclosafe.launch.py**](../../src/cyclosafe/README.md#launch-description)

S'il est activé (**sudo systemctl enable cyclosafed.service**), les mesures sont prises dès le démarrage du raspberry.

Il lance le script [`launch_wrapper.sh`](../../scripts/launch_wrapper.sh). Il s'agit d'un wrapper pour sourcer l'environnement ROS et executer la commande `ros2 launch cyclosafe cyclosafe.launch.py record:=true save:=false`.

> **Note** : ce service est dépendant du bon fonctionnement des services [**gpiod.service**](#gpiodservice) et [**gps_time.service**](#gps_timeservice).

> Si un des noeuds n'est pas correment lancé ou s'arrête, le service est relancé.

### Fermeture : SIGINT

Le signal utilisé pour la fermeture (**SIGINT**) est primordiale car il définit la façon dont la prise de mesure s'arrête.

En cas d'enregistrement des données (**record:=true**), il est essentiel que le programme rosbag puisse terminer de vider son cache et d'écrire le fichier ***metadata.yaml***.

Lors d'une fermeture brutale causée par un timeout de la procédure d'arrêt (délai ***TimeoutStopSec=6*** atteint) ou d'une coupure d'alimentation, il faut s'attendre à :
- le dernier fichier .mcap (celui en cours d'écritureau moment de l'arrêt) ne sera pas compressé (.mcap au lieu .mcap.zstd)
- une corruption possible du fichier .mcap en cours d'écriture (vérifiable avec **ros2 bag info ***chemin_du_dossier_bag*****)
- une absence du fichier **metadata.yaml** contenant les métadonnées sur le bag et référencant l'ensemble des différentes fichiers .mcap (compressés ou non). Après avoir décompresser l'ensemble des bags en .zstd, celui-ci peut-être reconstruit à l'aide de **ros2 bag reindex ***chemin_du_dossier_bag*****.

> Note: Le [**script d'import des enregistrements**](../../../scripts/README.md#réparation-des-corruptions) inclue une fonction permettant de réparer automatiquement les corruptions listées ci-dessus.

### Valeurs paramètrables

**SHUTDOWN_DELAY** : ajuste le temps imparti à l'arrêt normal du service avant de lui envoyer un **SIGTERM** ou un **SIGKILL** (causant une probable corruption de données). ***TimeoutStopSec*** étant défini à **SHUTDOWN_DELAY + 1**.

**CYCLOSAFE_RECORD** : définit le répertoire où les enregistrements sont sauvegardés

## gpiod.service

Service démarrant le script [**gpio.sh**](../../scripts/README#gpiosh).

Il est crucial que ce service démarre correctement car le script est responsable :

- du monitoring du bouton d'extinction
- du contrôle des leds
- du monitoring de la tension de la batterie
- du monitoring de l'espace restant sur la carte SD
- de l'arrêt propre de cyclosafed.service

## gps_time.service

Service démarrant le script [**gps_time.sh**](../../scripts/README#gps_timesh).

Le script est lancé une seule fois à chaque démarrage.

## pigpiod.service

Lance le daemon de pigpio sur lequel repose la libraire python permettant d'interagir avec les gpio du raspberry.
