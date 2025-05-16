#!/bin/bash

# Configuration
GPS_DEVICE="/dev/ttyACM0"  # À adapter selon votre port GPS
BAUDRATE=115200              # À adapter selon votre appareil GPS
MAX_ATTEMPTS=3             # Nombre maximum de tentatives
READ_TIMEOUT=3            # Délai d'attente pour la lecture en secondes
LOG_DIR="/home/npirard/data/logs"
LOG_FILE="/home/npirard/data/logs/gps_time_sync.log"

log() {
  echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a $LOG_FILE
}

sync_time_from_gps() {
  local attempt=1
  
  log "Starting date-time synchronisation with GPS"
  
  # Vérifier si le périphérique GPS existe
  if [ ! -e "$GPS_DEVICE" ]; then
    log "Error: GPS peripheral $GPS_DEVICE not found"
    return 1
  fi
  
   # Configurer le port série
   stty -F $GPS_DEVICE $BAUDRATE raw -echo

  while [ $attempt -le $MAX_ATTEMPTS ]; do
    log "Sync attempt $attempt/$MAX_ATTEMPTS"
    
    # Utiliser timeout pour limiter le temps de lecture
    local nmea_data=$(timeout $READ_TIMEOUT cat $GPS_DEVICE 2>/dev/null | grep -m 1 "\$GNRMC")
    
    if [ -z "$nmea_data" ]; then
      log "No data from GPS"
      ((attempt++))
      sleep 2
      continue
    fi
    
    log "GPS data received: $nmea_data"
    
    # Extraction de l'heure (format HHMMSS.SS)
    local gps_time=$(echo "$nmea_data" | cut -d',' -f2 | cut -d'.' -f1)
    
    # Extraction de la date (format DDMMYY)
    local gps_date=$(echo "$nmea_data" | cut -d',' -f10)
    
    if [ -z "$gps_time" ] || [ -z "$gps_date" ]; then
      log "Cannot extract date and/or time from gps data"
      ((attempt++))
      sleep 2
      continue
    fi
    
    # Format de l'heure: HHMMSS.SS
    local hours=${gps_time:0:2}
    local minutes=${gps_time:2:2}
    local seconds=${gps_time:4:2}
    
    # Format de la date: DDMMYY
    local day=${gps_date:0:2}
    local month=${gps_date:2:2}
    local year=20${gps_date:4:2}  # On ajoute "20" pour obtenir l'année complète
    
    log "Extracted: $hours:$minutes:$seconds, Date: $day/$month/$year"
    
    # Configuration de la date et l'heure du système
    local date_cmd="date -u -s \"$year-$month-$day $hours:$minutes:$seconds UTC\""
    log "EExecuting: $date_cmd"
    
    if eval $date_cmd; then
      log "System time updated"
      
      return 0
    else
      log "Fail to update system time"
      ((attempt++))
      sleep 2
    fi
  done
  
  log "Failure after $MAX_ATTEMPTS attempts"
  return 1
}

mkdir -p "$LOG_DIR"
touch "$LOG_FILE"

# Exécution de la fonction principale
sync_time_from_gps
exit_code=$?

log "Exit code: $exit_code"
exit $exit_code