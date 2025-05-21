#!/bin/bash

IF_NAME=eth0
CONFIG_NAME=Internet-DHCP

RASPI_IP=192.168.2.2
HOST_IP=192.168.2.1

echo Setting up nmcli config named "$CONFIG_NAME" on interface "$IF_NAME"

sudo nmcli connection add "$CONFIG_NAME" ifname "$IF_NAME" ipv4.method manual ipv4.addresses $RASPI_IP/24 ipv4.gateway $HOST_IP ipv4.dns "8.8.8.8,8.8.4.4"
sudo nmcli connection up "$CONFIG_NAME"

echo raspberry ip=$RASPI_IP
echo host ip=$HOST_IP

echo Import: make sure to config ip $HOST_IP on host !
echo To test connection to host: ping $HOST_IP
echo To test connection to internet: ping www.google.fr