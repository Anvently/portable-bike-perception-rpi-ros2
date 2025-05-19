#!/bin/bash

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
  echo $(color white '[INFO]') $1
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

if [ -f /opt/ros/jazzy/setup.bash ]; then
	info "Ros is already installed"
else
	info "-------------- Installing ROS2-Jazzy ----------------"
	if ! [ -f ./ros-jazzy-desktop-0.3.2_20240525_arm64.deb ]; then
		wget https://s3.ap-northeast-1.wasabisys.com/download-raw/dpkg/ros2-desktop/debian/bookworm/ros-jazzy-desktop-0.3.2_20240525_arm64.deb
		if ! [ $? -eq 0 ]; then
			error "Failed to download ROS2 .deb package"
			exit 1
		fi
	fi
	sudo apt install ./ros-jazzy-desktop-0.3.2_20240525_arm64.deb
	if ! [ $? -eq 0 ]; then
		error "Failed to install ROS2 .deb package"
		exit 1
	fi
fi

pip install --break-system-packages empy==3.3.4
if ! [ $? -eq 0 ]; then
	error "Failed to install empy"
	exit 1
fi

sudo pip install --break-system-packages vcstool colcon-common-extensions
if ! [ $? -eq 0 ]; then
	error "Failed to install dependencies"
	exit 1
fi

rosdep init
rosdep update

sudo apt update
if ! sudo apt-get install -y cmake curl libopencv-dev python3-pip python3-opencv python3-numpy python3-smbus python3-pigpiod >/dev/null 2>&1; then
    error "Failed to install dependencies but ros core was installed."
fi