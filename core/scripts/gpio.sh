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


LOG_DIR="$CYCLOSAFE_LOGS"
LOG_FILE="$CYCLOSAFE_LOGS/cyclosafe.log"
SCRIPT_DIR="$SCRIPTS_PATH"
PYTHON_SCRIPT="gpio.py"

echo $LOG_DIR $CYCLOSAFE_LOGS

mkdir -p "$LOG_DIR"
touch "$LOG_FILE"
chmod 777 $LOG_FILE
chmod 777 $LOG_DIR

# Fonction de logging avec niveau de gravité
log() {
	local level="$1"
	local message="$2"
	echo "$(date '+%Y-%m-%d %H:%M:%S') - [$level] $message" | tee -a "$LOG_FILE"
}

log "INFO" "Powered on"

cd $SCRIPT_DIR
export PYTHONPATH=$SCRIPT_DIR:$PYTHONPATH
python3 "$PYTHON_SCRIPT"
EXIT_CODE="$?"

# Vérifier si le retour est -1 ou -2
if [ $EXIT_CODE -eq 255 ]; then
	log "INFO" "Shutdown triggered by button"
elif [ $EXIT_CODE -eq 254 ]; then
	log "INFO" "Shutdown triggered by low battery"
else
	log "ERROR" "Unknown exit code $EXIT_CODE"
fi

systemctl stop cyclosafed.service
sleep $(($SHUTDOWN_DELAY + 1))

systemctl is-active --quiet service && log "WARNING" "Shutdown while service still active"

if [ $EXIT_CODE -eq 255 ] || [ $EXIT_CODE -eq 254 ]; then
	shutdown -h now
	# echo "SHUTDOWN"
else
	log "WARNING" "Shutdown cancelled"
fi