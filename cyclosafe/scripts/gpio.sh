#!/bin/bash

LOG_DIR="/home/npirard/data/logs"
LOG_FILE="/home/npirard/data/logs/on_off.log"
PYTHON_SCRIPT="/home/npirard/ros_ws/install/cyclosafe/share/cyclosafe/gpio.py"

mkdir -p "$LOG_DIR"
touch "$LOG_FILE"

echo "[INFO] $(date) - Powered on" | tee -a "$LOG_FILE"
python3 "$PYTHON_SCRIPT"
EXIT_CODE="$?"

# Vérifier si le retour est -1 ou -2
if [ $EXIT_CODE -eq 1 ]; then

	echo "[INFO] $(date) - Shutdown triggered by button" | tee -a "$LOG_FILE"

elif [ $EXIT_CODE -eq 2 ]; then

   echo "[INFO] $(date) - Shutdown triggered by low battery" | tee -a "$LOG_FILE"

else

	echo "[WARNING] $(date) - Unknown exit code $EXIT_CODE" | tee -a "$LOG_FILE"

fi

systemctl stop cyclosafed.service
sleep 3

systemctl is-active --quiet service && (echo "[WARNING] $(date) - Shutdown while service still active" | tee -a "$LOG_FILE")

shutdown -h now