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
  echo $(color green '[INFO]') $1
}

function error() {
  echo $(color red '[ERROR]') $1
}

function warning() {
  echo $(color yellow '[WARNING]') $1
}

parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
echo $parent_path
cd "$parent_path"

if [ "$EUID" -eq 0 ]; then
  error "This script must be run as normal user."
  exit 1
fi

uname -a | grep raspberrypi > /dev/null
if [ $? -eq 1 ]; then
  error "This device does not appear to be a raspberry pi"
  exit 1
fi

if [ -n "$SUDO_USER" ]; then
  USERNAME="$SUDO_USER"
else
  USERNAME=$USER
fi

env_file=$(realpath $parent_path/../.env)

HOME=$(eval echo ~$USERNAME)
source $env_file

SCRIPTS_PATH="$CYCLOSAFE_WORKSPACE/scripts"

sudo systemctl status cyclosafed.service > /dev/null
if [ $? -eq 4 ]; then
  info "Installing cyclosafed service"
  # Créer le fichier service
cat << EOF | sudo tee /etc/systemd/system/cyclosafed.service
[Unit]
Description=cyclosafe daemon
After=gpiod.service
Requires=gpiod.service
StartLimitIntervalSec=60
StartLimitBurst=3

[Service]
Type=simple
User=$USERNAME
Restart=on-failure
EnvironmentFile=$env_file
ExecStart=/bin/bash -c "source \$CYCLOSAFE_WORKSPACE/setup/.bashrc; ros2 launch cyclosafe cyclosafe.launch.py record:=true save:=false"
TimeoutStopSec=$SHUTDOWN_DELAY
KillMode=mixed
KillSignal=SIGINT
RestartKillSignal=SIGINT
SendSIGKILL=yes

[Install]
WantedBy=multi-user.target
EOF
fi

sudo systemctl status gpiod.service > /dev/null
if [ $? -eq 4 ]; then
  info "Installing gpiod service"
  # Créer le fichier service
cat << EOF | sudo tee /etc/systemd/system/gpiod.service
[Unit]
Description=gpio daemon controlling led blinking, button press and battery monitoring
After=pigpiod.service
Requires=pigpiod.service
StartLimitIntervalSec=60
StartLimitBurst=3

[Service]
Type=simple
User=root
Restart=on-failure
EnvironmentFile=$env_file
ExecStart=/bin/bash -c "source \$SCRIPTS_PATH/gpio.sh"
TimeoutStopSec=1
KillMode=mixed
KillSignal=SIGKILL
SendSIGKILL=yes

[Install]
WantedBy=multi-user.target
EOF
fi

sudo systemctl status gps_time.service > /dev/null
if [ $? -eq 4 ]; then
  info "Installing gps_time service"
  # Créer le fichier service
cat << EOF | sudo tee /etc/systemd/system/gps_time.service
[Unit]
Description=Synchronisation de l'heure système via GPS
After=network.target
Wants=network.target

[Service]
Type=oneshot
EnvironmentFile=$env_file
ExecStart=/bin/bash -c "source \$SCRIPTS_PATH/gps_time.sh"
RemainAfterExit=yes
StandardOutput=journal

[Install]
WantedBy=multi-user.target
EOF
fi

sudo systemctl status pigpiod.service > /dev/null
if [ $? -eq 4 ]; then
  info "Installing pigpiod service"
  # Créer le fichier service
cat << EOF | sudo tee /etc/systemd/system/pigpiod.service
[Unit]
Description=pigpio library daemon
After=network.target

[Service]
Type=simple
Restart=on-failure
ExecStart=/usr/bin/pigpiod -lg
TimeoutStopSec=1
KillMode=mixed
KillSignal=SIGKILL
SendSIGKILL=yes

[Install]
WantedBy=multi-user.target
EOF
fi

sudo systemctl daemon-reload

sudo systemctl enable pigpiod.service
sudo systemctl start pigpiod.service
info "pigpiod.service enabled and started"

sudo systemctl enable gps_time.service
sudo systemctl enable gps_time.service
info "gps_time.service enabled and started"

sudo systemctl enable gpiod.service
sudo systemctl start gpiod.service
info "gpiod.service enabled and started"

info "cyclosafed.service was not enabled. If you want to enable at startup, use: sudo systemctl enable cyclosafed.service."


