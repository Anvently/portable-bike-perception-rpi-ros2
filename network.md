# Configuration réseau pour ssh et ip-forwarding

Afin de faciliter les opérations, que ce soit pour exporter les données ou modifier l'environnement sur le raspberry, il est fortement recommandé d'accéder au raspberry via ssh.

Ce document donne des instructions sur :
- les commandes et outils permettant d'interagir avec le raspberry en ssh (une fois les configurations effectuées)
- les paramètres réseaux à configurer sur le raspberry et sur l'hôte pour permettre une connection ssh
- les paramètres réseaux à configurer sur l'hôte pour activer l'ip-forwarding sur l'hôte et permettre au raspberry d'utiliser l'accès internet de celui-ci.

### Structure réseau

Le sous-réseau qui va être configuré a pour addresse `192.168.2.1/24`, c'est à dire est l'ensemble des adresses commencant par `192.168.2.XXX`.

Dans ce sous-réseau, il y a deux appareils :
- l'hôte qui a pour adresse `192.168.2.1` et qui fait office de routeur vers internet
- le raspberry qui a pour addresse `192.168.2.2`

## Utilisation

Suppose que la configuration réseau est déjà active **[sur le raspberry](#mise-en-place-sur-le-raspberry) ET [sur l'hôte](#mise-en-place-sur-lhôte)**.

### Se connecter en ssh

~~~
ssh user@192.168.2.2
~~~

`user` est le nom d'utilisateur choisi lors de l'installation du raspberry.

### Copier des fichiers depuis l'hôte vers le raspberry

~~~
scp file_to_copy user@192.168.2.2:/home/user/out_path_on_raspberry

# Pour copier un dossier
scp -r folder_to_copy/ user@192.168.2.2:/home/user/out_path_on_raspberry
~~~

### Copier des fichiers depuis le raspberry vers l'hôte

~~~
scp user@192.168.2.2:/home/user/file_to_copy out_path_on_host

# Pour copier un dossier
scp -r user@192.168.2.2:/home/user/folder_to_copy out_path_on_host
~~~

## Mise en place sur le raspberry

### Configurer le profil sur le raspberry

1. Mettre en place le profil réseau
   ~~~
   sudo nmcli connection add Internet-DHCP ifname eth0 ipv4.method manual ipv4.addresses 192.168.2.2/24 ipv4.gateway 192.168.2.1 ipv4.dns "8.8.8.8,8.8.4.4"
   ~~~
2. Activer le profil réseau
   ~~~
   sudo nmcli con up Internet-DHCP
   ~~~


### Activer le serveur ssh

1. Vérifier dans raspi-config que le ssh est activé :
   1. Executer la commande `sudo raspi-config`
   2. Interface Options->SSH Enable/disable ....
   3. Fermer raspi-config
2. Activer le serveur ssh dans les services systemd
   1. Executer la commande `sudo systemctl enable ssh` : le serveur sera désormais lancé au démarrage du raspberry
   2. Démarrer le serveur ssh (sans avoir à redémarrer) : `sudo systemctl start ssh`
   3. Vérifier le statut du service : `sudo systemctl status ssh`

## Mise en place sur l'hôte

### Profil réseau

1. Identifier le nom de l'interface ethernet à laquelle sera branché le raspberry.
   ~~~
   ifconfig
   ~~~
   L'interface en question ressemble généralement à `eth0`, `eno1`, `enp0s3` ou similaire.

   <ins>**Note**</ins> : Si le port ethernet de l'hôte est déjà utilisé pour l'accès internet, vous pouvez connecter le raspberry au switch par lequel celui-ci est connecté au réseau internet. L'interface à identifier reste la même.
2. Mettre en place le profil réseau en substituant par le nom de l'interface identifiée
   ~~~
   sudo nmcli connection add "Static-raspi" ifname INTERFACE_NAME ipv4.method manual ipv4.addresses 192.168.2.1/24
   ~~~
3. Activer le profil réseau
   ~~~
   sudo nmcli con up Static-raspi
   ~~~

A partir de là, il est possible de brancher le raspberry en ethernet et de s'y connecter en ssh.

### ip-forwarding (sur l'hôte)

L'ip-forwarding permet à l'hôte de comporter comme un routeur et de faire les requêtes du raspberry vers internet en son nom.

Le prérequis est que le raspberry ait comme gateway ipv4 l'adresse de l'hôte (ce qui est normalement le cas si la procédure de configuration a été suivie).

#### Pour activer l'ip forwarding
   ~~~
   sudo sysctl -w net.ipv4.ip_forward=1
   ~~~
   Pour rendre le changement permanent :
   ~~~
   sudo nano /etc/sysctl.conf
   ~~~
   Et ajouter cette ligne :
   ~~~
   net.ipv4.ip_forward=1
   ~~~

<ins>**A partir de là, deux scénarios possibles</ins> :**

#### L'accès internet de l'hôte et la connexion au raspberry sont sur le même interface.

<ins>**Ex</ins>** : internet est obtenu sur l'hôte via internet et le raspberry est connecté sur le même réseau local via un switch.

Le raspberry devrait avoir accès à internet.

#### L'accès internet de l'hôte est obtenu via une autre interface (**ex**: wifi) que celle sur laquelle est connectée le raspberry.

Dans ce cas il est nécessaire de configurer les ip-tables afin de rediriger le traffic entre les deux interfaces.
1. Identifier le nom de l'interface wifi sur laquelle est obtenue internet.
   ~~~
   ifconfig
   ~~~
   L'interface en question ressemble généralement à `wlan0`, `wlp2s0` ou similaire.
   
   Une fois l'interface identifiée, vous pouvez enregistrer le nom des différentes interfaces dans des variables.
   ~~~
   ETHERNET_INTERFACE=enx34298f731b2e
   WIFI_INTERFACE=wlp2s0
   ~~~
2. Configurer le NAT avec iptables :
   ~~~
   sudo iptables -t nat -A POSTROUTING -o $WIFI_INTERFACE -j MASQUERADE
   sudo iptables -A FORWARD -i $ETHERNET_INTERFACE -o $WIFI_INTERFACE -j ACCEPT
   sudo iptables -A FORWARD -i $WIFI_INTERFACE -o $ETHERNET_INTERFACE -m state --state RELATED,ESTABLISHED -j ACCEPT
   ~~~
3. Pour rendre ces changements persistents, vous pouvez installer `iptables-persistent`.
   ~~~
   sudo apt install iptables-persistent
   ~~~
   Cela suppose un accès internet, mais à ce stade vous devriez y avoir accès.

   Pendant l'installation, il vous sera demandé si vous sera proposé d'enregistrer les règles actuelles.
### Activer le serveur ssh sur l'hôte

Cet étape n'est pas indispensable.

L'idée est de permettre au raspberry d'effectuer des requêtes ssh (ou scp) vers l'hôte.

Par exemple cela peut être utile pour indiquer comme remote git l'adresse du repo sur l'hôte (exemple `ssh://npirard@192.168.2.1:/home/npirard/cyclosafe_repo`).

Dans cette configuration là, l'hôte n'a qu'à ```git commit``` les changements et ils peuvent récupéré sur le raspberry via ```git pull```.

1. Installer le serveur ssh :
   1. Executer la commande `sudo apt install openssh-server`
2. Activer le serveur ssh dans les services systemd
   1. Executer la commande `sudo systemctl enable ssh` : le serveur sera désormais lancé au démarrage du raspberry
   2. Démarrer le serveur ssh (sans avoir à redémarrer) : `sudo systemctl start ssh`
   3. Vérifier le statut du service : `sudo systemctl status ssh`

