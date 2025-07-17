#!/bin/bash

if [ "$EUID" -ne 0 ]; then
  echo This script must be run as root. Use sudo
  exit 1
fi

sudo rm /etc/systemd/system/cyclosafed.service
sudo rm /etc/systemd/system/pigpiod.service
sudo rm /etc/systemd/system/gpiod.service
sudo rm /etc/systemd/system/gps_time.service

systemctl stop cyclosafed.service gps_time.service pigpiod.service gpiod.service
systemctl disable cyclosafed.service gps_time.service pigpiod.service gpiod.service
systemctl daemon-reload
systemctl reset-failed