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
# https://cecill.info/licences/Licence_CeCILL-B_V1-en.html

function color() {
  if [ "$#" -ne 2 ] ; then
    echo "[ERROR] color <color-name> <text> expected two arguments, but got $#" >&2
    return 1
  fi

  local -r colorName="$1"
  local -r message="$2"
  
  local colorCode="0;37"
  case "${colorName,,}" in
    black          ) colorCode='0;30' ;;
    red            ) colorCode='0;31' ;;
    green          ) colorCode='0;32' ;;
    yellow         ) colorCode='0;33' ;;
    blue           ) colorCode='0;34' ;;
    magenta        ) colorCode='0;35' ;;
    cyan           ) colorCode='0;36' ;;
    white          ) colorCode='0;37' ;;
    bright_black   ) colorCode='0;90' ;;
    bright_red     ) colorCode='0;91' ;;
    bright_green   ) colorCode='0;92' ;;
    bright_yellow  ) colorCode='0;93' ;;
    bright_blue    ) colorCode='0;94' ;;
    bright_magenta ) colorCode='0;95' ;;
    bright_cyan    ) colorCode='0;96' ;;
    bright_white   ) colorCode='0;97' ;;
    gray           ) colorCode='0;90' ;;
    *              ) colorCode='0;37' ;;
  esac
 
  echo -e "\e[${colorCode}m${message}\e[0m"
}

function info() {
  echo $(color green '[INFO]') $1
}

function error() {
  echo $(color red '[ERROR]') $1
}

function warning() {
  echo $(color yellow '[WARNING]') $1
}

parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
cd "$parent_path"


if [ "$EUID" -eq 0 ]; then
  error "This script must be run as normal user."
  exit 1
fi

info "Please log in"
sudo echo "dummy" > /dev/null

uname -a | grep raspberrypi > /dev/null
if [ $? -eq 1 ]; then
  error "This device does not appear to be a raspberry pi"
  exit 1
fi

if [ -n "$SUDO_USER" ]; then
  USERNAME="$SUDO_USER"
fi

HOME=$(eval echo ~$USERNAME)
source .env

mkdir -p $CYCLOSAFE_RECORD
mkdir -p $CYCLOSAFE_LOGS

if [ $(cat /boot/firmware/cmdline.txt | grep -c ipv6.disable=1) -eq 0 ]; then
  info "Updating boot config files".
  sudo cp /boot/firmware/cmdline.txt /boot/firmware/cmdline.txt.backup
  sudo cp /boot/firmware/config.txt /boot/firmware/config.txt.backup
  sudo tee -a /boot/firmware/cmdline.txt < ./cmdline.txt > /dev/null
  sudo tee -a /boot/firmware/config.txt < ./config.txt > /dev/null
  info "--- Boot config was back up and updated. You must reboot the raspberry pi before making any further installation." 
  exit 0
fi

if ! [ -f /opt/ros/jazzy/setup.bash ]; then
  info "No ROS2 installation detected. Installing." 
	./ros_install.sh
fi

info "Sourcing ROS2"

source /opt/ros/jazzy/setup.bash

cd $CYCLOSAFE_WORKSPACE

info "Installing dependencies"

rosdep install -ry --from-path src

if [ $? -ne 0 ]; then
  warning "Some ROS package dependency failed to install"
fi

info "Building ROS packages"
colcon build --symlink-install --parallel-workers=2

if [ $? -ne 0 ]; then
  warning "Build errors detected. Check build output for any fatal error."
  warning "You can use cy_core_build (after sourcing cyclosafe_environment) to retry the build."
fi

cd $parent_path
echo $parent_path

systemctl status cyclosafed.service > /dev/null
if [ $? -eq 4 ]; then
  info "Setting up cyclosafe services"
  ./systemd/setup_services.sh
fi

info "Installation is done. Make sure no error or warning show up in the logs."
info "You can source cyclosafe environment using : source $CYCLOSAFE_WORKSPACE/setup/.bashrc"
info "You can enable environment at startup using : \$(cat $CYCLOSAFE_WORKSPACE/setup/.bashrc) >> ~/.bashrc"
info "To start cyclosafe recordings you can use: sudo systemctl start cyclosafed.service"
