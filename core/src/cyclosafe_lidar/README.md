# cyclosafe_lidar

Package cpp qui permet d'intégrer les lidars de la gamme Benewake à ROS2. 

Implémente un driver via l'interface sériale et un noeud pour publier les données sous forme de `sensor_msgs/msg/Range`.

Compatible en **UART** avec les modèles suivants :

- [**TF02-Pro**](https://en.benewake.com/TF02Pro/index.html)
- [**TFmini Plus**](https://en.benewake.com/TFminiPlus/index.html)
- [**TFS20-L**](https://en.benewake.com/TFS20L/index.html) : non testé

Etant donné que le protocole utilisé par le fabricant est commun à l'ensemble des lidars (avec quelques différences pour certains paramètres), la majeur partie du code est probablement compatible avec le reste de la gamme.

Vérifier les datasheet pour voir ce qui peut différer.

## Usage

~~~
ros2 run cyclosafe_lidar node_lidar --ros-args -p port:=/dev/ttyS0 -p baud:=115200 -p period:=0.01 -p model:=tf-02 -p framerate:=100
~~~

Lance le noeud sur l'UART1 du raspberry sur lequel est connecté un lidar TF-02. Le `framerate` est volontairement proche de `period` afin de minimiser le crosstalk.

## Paramètres

> **port** : string
> - **défaut** : "/dev/ttyS0"
> - port sérial sur lequel est connecté le capteur

> **baud** : int
> - **défaut** : *115200*
> - baudrate du port sérial

> **period** : float
> - **défaut** : *0.01*
> - **défaut** : *secondes*
> - période à laquelle les frames envoyées par le capteur sont lues

> **model** : string
> - **défaut** : *""*
> - **accepte** : *"tf-02"*, *"tf-mini-plus"*, *"tf-s20l"*
> - modèle du lidar connecté. Permet de savoir quel `Benewake::ADriver` instancier.

> **trigger** : boolean
> - **défaut** : *false*
> - si `true`, le lidar fonctionnera en *trigger mode*, c'est à dire qu'il ne prend des mesures que sous la commande du driver. Permet théoriquement de limiter le cross-talk entre différents capteurs, pourvu qu'on sychronise la prise de mesure. Le `framerate` est automatiquement défini à 0.
> - Si `false`, le lidar fonctionnera en *free-running mode*, c'est à dire qu'il prend des mesures et envoie des mesures continuellement. Permet théoriquement d'atteindre un framerate plus elevé mais peut générer du cross-talk entre différents capteurs.

> **framerate** : int
> - **défaut** : 100
> - **min** : *0*
> - **max** : *250* à *1000* selon le modèle
> - **unit** : hz
> - fréquence à laquelle configurer le capteur, détermine la vitesse à laquelle celui-ci prend les mesure et envoie les frames
> - `framerate=0` et `trigger=true` correspond à une configuration en trigger mode
> - `framerate=0` et `trigger=false` correspond à une désactivation du capteur

> **target_baud** : int
> - **défaut** : *115200*
> - baudrate sur lequel reconfigurer le port sérial au démarrage du noeud. Permet d'accélérer le débit lorsqu'on utilise une fréquence élevée.
> - ***Attention*** : ce changement persiste à la fermeture du noeud, tant que le capteur est alimenté.
> - ***Attention*** : pas suffisamment testé

## Mode de contrôle

### Trigger mode (trigger=true)

Le `framerate` est défini à 0 est l'interval de mesure est cadencé sur le paramètre `period`.

Permet de réduire le crosstalk.

### Free-running mode (trigger=false et framerate > 0)

Le capteur prend une mesure et les envoie à une fréquence définie par `framerate`.

Les frame sont lues sur l'interface serial toutes les `period` secondes.

Si la fréquence de mesure est plus grande que la fréquence de lecture (`framerate > (1  / period)`), une lecture peut décoder plusieurs frames envoyées par le capteur, ce qui aboutira à la publication de plusieurs messages. Cette situation peut donner lieu à une fréquence de publication irrégulière sur le topic **/range**.

## Intensité retour et valeurs incertaines

Les éblouisssement (`strength = 65535`), les valeurs incertaines (`strength < 100`) ou les distances nulles (`distance = 0`) sont publiés avec une distance valant `NaN`.

## Serial interface

La classe `Serial.hpp` défini une API utilisée par le driver permettant de contrôler l'interface sériale et de lire/écrire dessus.

Elle repose sur une communication non bloquante (le flag `O_NONBLOCK` est activé sur le `fd` du port) qui fonctionne en **polling** avec un timeout assignés à chaque opération.

Plusieurs noeuds `node_lidar` ne peuvent utiliser le même port puisque celui-ci est lock via un fichier `/var/lock/LCK..dev_name` à l'initialisation.

## Benewake driver

Le namespace `Benewake` définit et implémente un ensemble de classes propres à l'utilisation de la gamme de lidars Benewake.

### Benewake::frames

- `struct Benewake::frames::DataFrame` : structure permettant de parser les frames de mesures envoyées par les capteurs.
- `struct Benewake::frames::CommandFrame` : structure permettant de construire les frames de commande
- `struct Benewake::frames::ResponseFrame` : structure permettant de parser les frames en réponse à une commande

### Benewake::ADriver

Classe abstraite contenant l'ensemble des méthodes communes aux différents modèles (du moins aux 3 modèles tf-02, tf-s20l et tf-mini-plus).

Déclare les méthodes abstraites suivantes :
- `virtual void	setFrameRate(unsigned int rate) const = 0`
- `virtual std::string getModelName() const = 0`
- `virtual std::string getFov() const` (non abstraite mais peut-être pertinent à réimplémenter dans une classe enfant)

De cette classe hérite :
- `Benewake::TF02Driver`
- `Benewake::TFMiniPlusDriver`
- `Benewake::TFS20LDriver`

