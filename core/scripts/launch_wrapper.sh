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


set -e
LOG_DIR="$CYCLOSAFE_LOGS"
LOG_FILE="$CYCLOSAFE_LOGS/cyclosafe.log"
echo $LOG_DIR $CYCLOSAFE_LOGS
mkdir -p "$LOG_DIR"
touch "$LOG_FILE"
source "$CYCLOSAFE_WORKSPACE/setup/.bashrc"

# Fonction de logging avec niveau de gravité
log() {
	local level="$1"
	local message="$2"
	echo "$(date '+%Y-%m-%d %H:%M:%S') - [$level] $message" | tee -a "$LOG_FILE"
}

# Vérification de l'espace disque disponible
DISK_USAGE=$(df -BM / | awk 'NR==2{print $4}' | sed 's/M//')
RECORD_OPTION="true"
EXPECTED_NODES=5

if [ "$ENCRYPTION" = "1" ]; then
	ENCRYPT_OPTION="true"
	EXPECTED_NODES=$((EXPECTED_NODES + 1))
	log "INFO" "Encryption enabled. Expected nodes increased to $EXPECTED_NODES"
fi

if [ $DISK_USAGE -lt $LOW_STORAGE_TRESHOLD ]; then
	log "INFO" "Low storage detected ($DISK_USAGE MB available). Recording disabled"
	RECORD_OPTION="false"
	EXPECTED_NODES=$((EXPECTED_NODES - 1))
else
	log "INFO" "Storage OK ($DISK_USAGE MB available). Recording enabled"
fi
# Gestionnaire de signal SIGINT
cleanup() {
	log "INFO" "Stopped via SIGINT"
	if [ -n "$LAUNCH_PID" ] && kill -0 $LAUNCH_PID 2>/dev/null; then
		log "INFO" "Terminating ros2 launch process..."
		kill -SIGINT $LAUNCH_PID
		wait $LAUNCH_PID 2>/dev/null || true
		log "INFO" "Gracefull shutdown of cyclosafe completed."
	fi
	exit 0
}

# Intercepter SIGINT
trap cleanup SIGINT

if [ "$ENCRYPT_OPTION" -eq 1 ]; then
    ros2 launch cyclosafe cyclosafe.launch.py record:=$RECORD_OPTION encrypt:=$ENCRYPT_OPTION &
else
    ros2 launch cyclosafe cyclosafe.launch.py record:=$RECORD_OPTION &
fi

LAUNCH_PID=$!
CHECK_INTERVAL=30

# Fonction pour vérifier les nœuds actifs
check_nodes() {
	local active_nodes_first
	local active_nodes_second
	
	active_nodes_first=$(ros2 node list | wc -l)
	
	sleep 1
	
	active_nodes_second=$(ros2 node list | wc -l)
	
	if [ "$active_nodes_first" -lt "$EXPECTED_NODES" ] && [ "$active_nodes_second" -lt "$EXPECTED_NODES" ]; then
		log "WARNING" "Expected $EXPECTED_NODES nodes, but only $active_nodes_first/$active_nodes_second found in consecutive checks. Restarting..."
		kill -SIGINT $LAUNCH_PID
		exit 1
	elif [ "$active_nodes_first" -lt "$EXPECTED_NODES" ] || [ "$active_nodes_second" -lt "$EXPECTED_NODES" ]; then
		# Si seulement une des deux vérifications échoue, on log un avertissement mais on continue
		log "INFO" "Node count fluctuation detected: $active_nodes_first/$active_nodes_second (expected: $EXPECTED_NODES). Continuing monitoring..."
	fi
}

# Boucle de vérification tant que ros2 launch est en vie
log "INFO" "Cyclosafe launch config running (record=$RECORD_OPTION, expected nodes=$EXPECTED_NODES)"
while kill -0 $LAUNCH_PID 2>/dev/null; do
	sleep "$CHECK_INTERVAL"
	check_nodes
done

# Attendre la fin du ros2 launch (si jamais on sort de la boucle sans l'avoir tué)
wait $LAUNCH_PID