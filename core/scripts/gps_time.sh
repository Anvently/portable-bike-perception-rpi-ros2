#!/bin/bash
# Configuration
GPS_DEVICE=$GPS_SERIAL_PORT # À adapter selon votre port GPS
BAUDRATE=115200 # À adapter selon votre appareil GPS
MAX_ATTEMPTS=3 # Nombre maximum de tentatives
READ_TIMEOUT=3 # Délai d'attente pour la lecture en secondes
LOG_DIR=$CYCLOSAFE_LOGS
LOG_FILE="$LOG_DIR/gps_time_sync.log"

# Validation des dates (format YYYY-MM-DD)
MIN_DATE="2025-07-17"  # Date minimale acceptable
MAX_DATE="2055-07-17"  # Date maximale acceptable

mkdir -p "$LOG_DIR"
touch "$LOG_FILE"
chmod 777 $LOG_FILE
chmod 777 $LOG_DIR

log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a $LOG_FILE
}

# Fonction pour valider si une date est dans la plage acceptable
validate_date() {
    local date_to_check="$1"
    
    # Convertir les dates en timestamps pour comparaison
    local date_timestamp=$(date -d "$date_to_check" +%s 2>/dev/null)
    local min_timestamp=$(date -d "$MIN_DATE" +%s 2>/dev/null)
    local max_timestamp=$(date -d "$MAX_DATE" +%s 2>/dev/null)
    
    # Vérifier si la conversion a réussi
    if [ -z "$date_timestamp" ] || [ -z "$min_timestamp" ] || [ -z "$max_timestamp" ]; then
        log "Error: Invalid date format for validation"
        return 1
    fi
    
    # Vérifier si la date est dans la plage acceptable
    if [ "$date_timestamp" -ge "$min_timestamp" ] && [ "$date_timestamp" -le "$max_timestamp" ]; then
        return 0
    else
        return 1
    fi
}

sync_time_from_gps() {
    local attempt=1
    log "Starting date-time synchronisation with GPS"
    log "Date validation range: $MIN_DATE to $MAX_DATE"
    
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
            log "Cannot extract date and/or time from GPS data"
            ((attempt++))
            sleep 2
            continue
        fi
        
        # Vérifier que les données ont la bonne longueur
        if [ ${#gps_time} -ne 6 ] || [ ${#gps_date} -ne 6 ]; then
            log "Invalid GPS time/date format: time=$gps_time (${#gps_time} chars), date=$gps_date (${#gps_date} chars)"
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
        local year=20${gps_date:4:2} # On ajoute "20" pour obtenir l'année complète
        
        log "Extracted: $hours:$minutes:$seconds, Date: $day/$month/$year"
        
        # Valider les composants de date/heure
        if [ "$hours" -lt 0 ] || [ "$hours" -gt 23 ] || \
           [ "$minutes" -lt 0 ] || [ "$minutes" -gt 59 ] || \
           [ "$seconds" -lt 0 ] || [ "$seconds" -gt 59 ] || \
           [ "$day" -lt 1 ] || [ "$day" -gt 31 ] || \
           [ "$month" -lt 1 ] || [ "$month" -gt 12 ]; then
            log "Invalid time/date components detected"
            ((attempt++))
            sleep 2
            continue
        fi
        
        # Formatter la date pour validation (YYYY-MM-DD), $((10#$year)) sert à forcer bash à interpréter les valeurs comme des décimales plutôt que des octales
        local formatted_date=$(printf "%04d-%02d-%02d" "$((10#$year))" "$((10#$month))" "$((10#$day))")
        
        # Valider la date
        if ! validate_date "$formatted_date"; then
            log "Date validation failed: $formatted_date is outside acceptable range ($MIN_DATE to $MAX_DATE)"
            ((attempt++))
            sleep 2
            continue
        fi
        
        log "Date validation passed: $formatted_date"
        
        # Configuration de la date et l'heure du système
        local date_cmd="date -u -s \"$year-$month-$day $hours:$minutes:$seconds UTC\""
        log "Executing: $date_cmd"
        
        if eval $date_cmd; then
            log "System time updated successfully to: $(date -u)"
            return 0
        else
            log "Failed to update system time"
            ((attempt++))
            sleep 2
        fi
    done
    
    log "Failure after $MAX_ATTEMPTS attempts"
    return 1
}

# Créer le répertoire de logs s'il n'existe pas
mkdir -p "$LOG_DIR"
touch "$LOG_FILE"

# Exécution de la fonction principale
sync_time_from_gps
exit_code=$?

log "Exit code: $exit_code"
exit $exit_code