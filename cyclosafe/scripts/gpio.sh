#!/bin/bash

LOG_FILE="/home/$USER/data/logs/on_off.log"
PYTHON_SCRIPT="/home/$USER/ros_ws/install/cyclosafe/share/cyclosafe/gpio.sh"

python3 "$PYTHON_SCRIPT"

# Vérifier si le retour est -1 ou -2
if [ "$?" -eq -1 ]; then

	echo "[INFO] $(date) - Shutdown triggered by button" | tee -a "$LOG_FILE"

elif [ "$?" -eq -2 ]; then

   echo "[INFO] $(date) - Shutdown triggered by low battery" | tee -a "$LOG_FILE"

fi

systemctl stop cyclosafed.service
sleep 3

systemctl is-active --quiet service && (echo "[WARNING] $(date) - Shutdown while service still active" | tee -a "$LOG_FILE")

shutdown -h now