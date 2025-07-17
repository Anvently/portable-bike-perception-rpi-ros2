#!/bin/bash
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

if [ $DISK_USAGE -lt $LOW_STORAGE_TRESHOLD ]; then
	log "INFO" "Low storage detected ($DISK_USAGE MB available). Recording disabled"
	RECORD_OPTION="false"
	EXPECTED_NODES=4
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
	fi
	exit 0
}

# Intercepter SIGINT
trap cleanup SIGINT

ros2 launch cyclosafe cyclosafe.launch.py record:=$RECORD_OPTION save:=false &
LAUNCH_PID=$!
CHECK_INTERVAL=30

# Fonction pour vérifier les nœuds actifs
check_nodes() {
	local active_nodes
	active_nodes=$(ros2 node list | wc -l)
	if [ "$active_nodes" -lt "$EXPECTED_NODES" ]; then
		log "WARNING" "Expected $EXPECTED_NODES nodes, but only $active_nodes found. Restarting..."
		kill -SIGINT $LAUNCH_PID
		exit 1
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