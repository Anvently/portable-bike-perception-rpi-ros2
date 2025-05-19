#!/bin/bash

LOG_DIR="$CYCLOSAFE_LOGS"
LOG_FILE="$CYCLOSAFE_LOGS/on_off.log"
SCRIPT_DIR="$SCRIPTS_PATH"
PYTHON_SCRIPT="gpio.py"

mkdir -p "$LOG_DIR"
touch "$LOG_FILE"

echo "[INFO] $(date) - Powered on" | tee -a "$LOG_FILE"

cd $SCRIPT_DIR
export PYTHONPATH=$SCRIPT_DIR:$PYTHONPATH
python3 "$PYTHON_SCRIPT"
EXIT_CODE="$?"

# Vérifier si le retour est -1 ou -2
if [ $EXIT_CODE -eq 255 ]; then

	echo "[INFO] $(date) - Shutdown triggered by button" | tee -a "$LOG_FILE"

elif [ $EXIT_CODE -eq 254 ]; then

   echo "[INFO] $(date) - Shutdown triggered by low battery" | tee -a "$LOG_FILE"

else

	echo "[ERROR] $(date) - Unknown exit code $EXIT_CODE" | tee -a "$LOG_FILE"

fi

systemctl stop cyclosafed.service
sleep $(($SHUTDOWN_DELAY + 1))

systemctl is-active --quiet service && (echo "[WARNING] $(date) - Shutdown while service still active" | tee -a "$LOG_FILE")

if [ $EXIT_CODE -eq 255 ] || [ $EXIT_CODE -eq 254 ]; then

	shutdown -h now
	# echo "SHUTDOWN"
else
	echo "[WARNING] $(date) - Shutdown cancelled" | tee -a "$LOG_FILE"
fi