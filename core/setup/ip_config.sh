#!/bin/bash
# Created on Tue Aug 05 2025
# Updated on Tue Aug 05 2025
#
#  This file is part of Cyclosafe
# Copyright (c) 2025 Nicolas Pirard @Anvently
#
# This software is governed by the CeCILL license under French law and
# abiding by the rules of distribution of free software. You can use,
# modify and/or redistribute the software under the terms of the CeCILL
# license as circulated by CEA, CNRS and INRIA at:
# https://cecill.info/licences/Licence_CeCILL_V2.1-en.html


IF_NAME=eth0
CONFIG_NAME=Internet-DHCP

RASPI_IP=192.168.2.2
HOST_IP=192.168.2.1

echo Setting up nmcli config named "$CONFIG_NAME" on interface "$IF_NAME"

sudo nmcli connection add type ethernet con-name "$CONFIG_NAME" ifname "$IF_NAME" ipv4.method manual ipv4.addresses $RASPI_IP/24 ipv4.gateway $HOST_IP ipv4.dns "8.8.8.8,8.8.4.4"
sudo nmcli connection up "$CONFIG_NAME"

echo raspberry ip=$RASPI_IP
echo host ip=$HOST_IP

echo Import: make sure to config ip $HOST_IP on host !
echo To test connection to host: ping $HOST_IP
echo To test connection to internet: ping www.google.fr