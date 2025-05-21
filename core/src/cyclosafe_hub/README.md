# cyclosafe_hub

Noeud en cpp qui a pour but d'aggréger des données venant de différents topics et de les sérialiser au format csv dans des fichiers isolés. Commande également l'enregistrement des photos via un service adressé au noeud camera.

Rendu obsolete par l'utilisation de rosbag, qui gère l'enregistrement, la sérialisation et la compression de toutes les données publiées.

## Usage

~~~
ros2 run cyclosafe_hub hub --ros-args -p out_path:=./record_folder -p cache_ttl:=30.0 -p save_interval:=3.0
~~~

Démarre le noeud avec un **cache de 30s** et un **interval d'écriture de 3s**. Les fichiers de sortie seront enregistrés dans le dossier du répertoire courant **record_folder/**.

## Fonctionnement

Le noeud souscrit aux topics **/gps**, **/range**, **/scan** et **/images**.

Chaque message reçu est enregistré dans un `TTLDeque<T>`, `T` étant la classe représentant le message.

A chaque **interval d'écriture** (donné par le paramètre **save_interval**), les données sont sérialisées au format csv dans le fichier correspondant (fichier du même nom que le topic). Le cache est vidé à chaque écriture.

Le format est le suivant:
~~~
sensor_msgs::msg::NavSatFix (topic /gps)

| timestamp |       latitude     | longitude | hdop |

sensor_msgs::msg::Range (topic /range)

| timestamp |     range (m)      |

sensor_msgs::msg::LaserScan (topic /scan)

| timestamp | angle_min (radian) | angle_max (radian) | angle_increment (radian) | scan_duration (s) | [...values]
~~~

Les images sont enregistrés dans un sous-dossier `images/`, le nom du fichier indiquant son timestamp.

### Timestamp

Tous les timestamp utilisés sont relatifs au début de l'enregistrement, donné par le paramètre `start_time`.

## Paramètres

> **cache_ttl** : float
> - **défaut** : *30.0*
> - **unité** : *secondes*
> - définit la durée de vie (*time_to_live*) des données mises en cache. Par défaut, les données des 30 dernières secondes seront conservées en mémoire.

> **save_interval** : float
> - **défaut** : *2.0*
> - **unité** : *secondes*
> - interval auquel les données sont sérialisées dans les fichiers

> **out_path** : string
> - **défaut** : *""*
> - chemin du dossier vers lequel enregistrer les données

> **start_time** : float
> - donne un point de référence vers le début de l'enregistrement pour timestamper les données.
> - **défaut** : temps de lancement du noeud
> - **unité** : secondes depuis l'epoch

## TTLDeque\<T\>

Le noeud repose sur l'utilisation de la classe template `TTLDeque<T>`, déifinie dans ***include/TTLDeque.hpp*** et associée à une valeur `_ttl` (*time_to_live*) qui indique la durée de vie des informations qu'elle contient.

Il revient à l'utilisateur de déterminer à quel moment le cache doit être vérifié et purgé des données expirées via la fonction `cleanup()`.

En l'état, le cache est purgé via `cleanup()` avant chaque écriture dans le fichier et il est `clear()` une fois l'écriture terminé. Cela signifie que le `_ttl` configuré n'a aucune influence sur le fonctionnement du noeud tant que `save_interval < cache_ttl`.

L'objectif de cette classe est surtout de permettre l'éventuelle implémentation d'un enregistrement de données à posteriori si elles sont jugées d'intérêt par le noeud ou un système de monitoring extérieur.

## Enregistrement des images

Contrairement aux autres types de données et pour des raisons de performance, les images ne sont pas récupérées via une souscription au topic correspondant. Ce travail est délégué au ***noeud caméra*** via le service [`cyclosafe_interfaces/srv/SaveImages`](../cyclosafe_interfaces/README.md#srvsaveimages).

La demande envoyée indique `out_path/images/`  comme dossier d'enreigstrement des images et demande d'enregistrer les images sur les `save_interval` dernières secondes.

[**Voir le noeud caméra pour plus d'infos**](../cyclosafe/README.md#acamera)

## TODO

- En l'état le noeud n'est pas compatible avec la launch description [**cyclosafe.launch.py**](../cyclosafe/README.md#launch-description) car le nom des topics auxquels souscrire est figé dans le code.

	Une approche simple pour pallier à ce problème serait d'ajouter un argument (une liste de strings) contenant les topics auxquels souscrire.

	Chaque topic serait enregistré dans un fichier dédié et le type de meesage `T` déterminerait la nature du `TTLDeque<T>` à utiliser.

	Etant donné qu'il peut y avoir plusieurs `TTLDeque<sensor_msgs::msg::LaserScan>` ou `TTLDeque<sensor_msgs::msg::Range>` (car plusieurs noeuds-capteurs peuvent publier le même type de message sur des topics différents), les membres de la classe devraient plutôt être des vecteurs.

- Le type de message attendu pour les positions gps est `sensor_msgs/msg/NavSatFix`, or [**le noeud gps**](../cyclosafe/README.md#gpspy) publie les données en **cyclosafe_interfaces/msg/NavSatInfo**, qui fournit davantage d'informatons.

	Il suffit de changer le type de message utilisé et compléter la fonction `std::ostream&	operator<<(std::ostream& os, const sensor_msgs::msg::NavSatFix& msg)` dans ****hub.cpp**** pour ajouter les informations manquantes à la sérialisation.