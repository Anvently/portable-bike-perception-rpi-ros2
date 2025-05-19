# Core scripts

Contient les scripts destinés à tourner sur le raspberry.

Le chemin vers ces scripts doit être indiqué par la variable d'environnement **SCRIPTS_PATH**.

## gpio.py

Script ayant plusieurs fonctions :

- monitoring du bouton d'extinction : Retourne 255 (-1) lors de l'appui du bouton
- contrôle des leds : clignotement des led
- monitoring de la batterie : mesure la tension toutes les 10s. Retourne 254 (-2) si la tension est inférieure à la tension minimale.

### Valeurs par défaut (paramètrables dans gpio.py)

- **BTN_RST_GPIO=16** : gpio pin auquel est connecté le bouton d'arrêt. Configuré au démarrage du noeud (et dans ***/boot/firmware/config.txt***) comme étant en pull-up (un appui correspond à une tension aux bornes du pin de 0V)
- **BUTTON_SHUTDOWN=255** : valeur de retour (alias code d'erreur) pour un appui sur le bouton. **NE PAS MODIFIER (à moins de corriger le script gpio.sh en conséquence)**
- **BUTTON_SHUTDOWN=254** : valeur de retour (alias code d'erreur) pour une batterie faible (voltage trop faible). **NE PAS MODIFIER (à moins de corriger le script gpio.sh en conséquence)**
- **NBR_CELLS=2** : nombre d'accus dans la batterie
- **CHARGE_VOLTAGE=4.2** : tension de charge d'un accu, cette valeur * le nombre d'accus définit la tension maximale de la batterie (lorsqu'elle est à 100%)
- **MIN_VOLTAGE=3** : tension minimale d'un accu.

**Ex:** Avec nbr_cells=2 et min_voltage=3, la tension de seuil à partir de laquelle le raspberry est éteint est de 2*3=6V

## gpio.sh

Script lancé au démarrage par [**gpiod.service**](../setup/systemd/README#gpiodservice).

Service ayant plusieurs fonctions :

- log des heures de démarrage et d'extinction du raspberry dans *****$CYCLOSAFE_LOGS**/on_off.log*** + cause de l'extinction (batterie ou bouton)
- lance le script python (**gpio.py**)
- test la valeur de retour du script et déclenche l'arrêt du raspbbery ssi le code d'erreur correspond à un appui sur le bouton ou à un voltage trop faible sur la batterie.
- déclenche l'arrêt du raspberry via ***sudo systemctl stop cyclosafe.service***. Respecte le délai configuré (*$SHUTDOWN_DELAY*), vérifie que le service a bien été arrêté (log de warning le cas échéant) et passe le raspberry en mode halt (extinction du bus d'alimentation, du bus USB, et consommation minimale de 23mA environ).

### Variables d'environnement liées

**SHUTDOWN_DELAY** : ajuste le temps du sleep  entre la commande d'arrêt du service cyclosafed.service et l'extinction du raspberry. La valeur du sleep étant définie à **SHUTDOWN_DELAY + 1**.

**CYCLOSAFE_LOGS** : définit le répertoire où on_off.log est enregistré

## battery_monitor.py

Implémente un driver python de la puce INA219 permettant de prendre des mesures de la tension de batterie et de consommation instantanée. Utilisé par **gpio.py**.

Basé sur : https://github.com/DFRobotdl/INA219-Python
Modifs:
- ajout d'une fonction get_battery_pourcent

Peut-être lancé seul pour afficher différentes informations sur la batterie et sur la puissance instantanée : ***python3 ./battery_monitor.py***

## gps_time.sh

Script lancé au démarrage par [**gps_time.service**](../setup/systemd/README#gps_timeservice).

Mets à jour l'heure et la date du système à partir des frames NMEA envoyées par un GPS connecté en serial.

### Variables d'environnement liées

**CYCLOSAFE_LOGS** : le script log ses tentatives dans ***$CYCLOSAFE_LOGS/gps_time_sync.log***

**GPS_SERIAL_PORT** : port serial auquel est connecté le GPS.



