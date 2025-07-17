#!/bin/bash

# Script de flashage de firmware avec calcul d'espace disponible
# Usage: ./flash_firmware.sh <fichier_image.xz> <device>
# Exemple: ./flash_firmware.sh cyclosafe_firmware.img.xz /dev/sdc

set -e  # Arrêter le script en cas d'erreur

# Vérifier si le script est lancé en tant que root
if [ "$EUID" -ne 0 ]; then
    echo "⚠️  Avertissement: Ce script n'est pas lancé en tant que root"
    echo "   Certaines opérations nécessiteront sudo"
    echo ""
fi

# Configuration par défaut (peut être surchargée par variable d'environnement)
RECORDING_DATA_RATE_MB_PER_MIN=${RECORDING_DATA_RATE_MB_PER_MIN:-7}

# Fonction d'aide
show_help() {
    echo "Usage: $0 <fichier_image.xz> <device>"
    echo ""
    echo "Arguments:"
    echo "  fichier_image.xz    Fichier image compressé à flasher"
    echo "  device              Device cible (ex: /dev/sdc)"
    echo ""
    echo "Variables d'environnement:"
    echo "  RECORDING_DATA_RATE_MB_PER_MIN    Débit d'enregistrement en MB/min (défaut: 7)"
    echo ""
    echo "Exemple:"
    echo "  $0 cyclosafe_firmware.img.xz /dev/sdc"
    echo "  RECORDING_DATA_RATE_MB_PER_MIN=128 $0 firmware.img.xz /dev/sdc"
}

# Vérification des arguments
if [ $# -ne 2 ]; then
    echo "Erreur: Arguments manquants"
    show_help
    exit 1
fi

IMAGE_FILE="$1"
DEVICE="$2"

# Vérifications de sécurité
if [ ! -f "$IMAGE_FILE" ]; then
    echo "Erreur: Le fichier image '$IMAGE_FILE' n'existe pas"
    exit 1
fi

if [ ! -b "$DEVICE" ]; then
    echo "Erreur: Le device '$DEVICE' n'existe pas ou n'est pas un device bloc"
    exit 1
fi

# Fonction pour démonter les partitions
unmount_partitions() {
    local device="$1"
    local mounted_partitions=()
    
    # Trouver toutes les partitions montées du device
    while IFS= read -r line; do
        if [[ "$line" =~ ^($device[0-9]*) ]]; then
            mounted_partitions+=("${BASH_REMATCH[1]}")
        fi
    done < <(mount | grep "$device")
    
    if [ ${#mounted_partitions[@]} -eq 0 ]; then
        return 0
    fi
    
    echo "🔍 Partitions montées:"
    for partition in "${mounted_partitions[@]}"; do
        mount_point=$(mount | grep "$partition" | awk '{print $3}')
        echo "   $partition → $mount_point"
    done
    
    read -p "Démonter automatiquement? (y/n): " auto_unmount
    
    if [ "$auto_unmount" = "y" ]; then
        echo "🔄 Démontage..."
        for partition in "${mounted_partitions[@]}"; do
            if sudo umount "$partition" 2>/dev/null; then
                echo "   ✅ $partition"
            else
                if sudo umount -f "$partition" 2>/dev/null; then
                    echo "   ✅ $partition (forcé)"
                else
                    echo "   ❌ Échec: $partition"
                    exit 1
                fi
            fi
        done
    else
        echo "❌ Démontez manuellement avant de continuer"
        exit 1
    fi
}

# Vérification et démontage des partitions si nécessaire
if mount | grep -q "$DEVICE"; then
    unmount_partitions "$DEVICE"
fi

# Confirmation avant flashage
echo "⚠️  ATTENTION: Effacement complet de $DEVICE"
echo "Image: $IMAGE_FILE"
echo "Débit: ${RECORDING_DATA_RATE_MB_PER_MIN} MB/min"
read -p "Continuer? (y/n): " confirmation

if [ "$confirmation" != "y" ]; then
    echo "❌ Opération annulée"
    exit 0
fi

# Obtenir la taille totale de la carte SD avant flashage
echo "📏 Taille totale de la carte SD..."
TOTAL_SD_SIZE_BYTES=$(blockdev --getsize64 "$DEVICE")
TOTAL_SD_SIZE_MB=$((TOTAL_SD_SIZE_BYTES / 1024 / 1024))
echo "   ${TOTAL_SD_SIZE_MB} MB"

# Flashage du firmware
echo "🔥 Flashage en cours..."
if xz -dc "$IMAGE_FILE" | sudo dd of="$DEVICE" bs=4M status=progress; then
    echo "✅ Flashage terminé"
else
    echo "❌ Erreur lors du flashage"
    exit 1
fi

# Synchronisation
echo "🔄 Synchronisation..."
sync

# Calculer la taille utilisée par les partitions
echo "📊 Calcul de l'espace utilisé..."
USED_SPACE_BYTES=0

# Parcourir toutes les partitions du device
for partition in "${DEVICE}"*; do
    if [ -b "$partition" ] && [ "$partition" != "$DEVICE" ]; then
        PARTITION_SIZE=$(blockdev --getsize64 "$partition" 2>/dev/null || echo "0")
        USED_SPACE_BYTES=$((USED_SPACE_BYTES + PARTITION_SIZE))
    fi
done

# Si aucune partition n'est détectée, essayer avec fdisk
if [ $USED_SPACE_BYTES -eq 0 ]; then
    LAST_SECTOR=$(sudo fdisk -l "$DEVICE" 2>/dev/null | grep "^${DEVICE}" | tail -1 | awk '{print $3}')
    if [ -n "$LAST_SECTOR" ]; then
        SECTOR_SIZE=$(sudo fdisk -l "$DEVICE" 2>/dev/null | grep "^Sector size" | awk '{print $4}')
        if [ -n "$SECTOR_SIZE" ]; then
            USED_SPACE_BYTES=$((LAST_SECTOR * SECTOR_SIZE))
        fi
    fi
fi

USED_SPACE_MB=$((USED_SPACE_BYTES / 1024 / 1024))
AVAILABLE_SPACE_MB=$((TOTAL_SD_SIZE_MB - USED_SPACE_MB))

echo "📈 Espace total: ${TOTAL_SD_SIZE_MB} MB"
echo "   Utilisé: ${USED_SPACE_MB} MB"
echo "   Disponible: ${AVAILABLE_SPACE_MB} MB"

# Calcul du temps d'enregistrement
if [ $AVAILABLE_SPACE_MB -gt 0 ] && [ $RECORDING_DATA_RATE_MB_PER_MIN -gt 0 ]; then
    RECORDING_MINUTES=$((AVAILABLE_SPACE_MB / RECORDING_DATA_RATE_MB_PER_MIN))
    RECORDING_HOURS=$((RECORDING_MINUTES / 60))
    RECORDING_MINUTES_REMAINDER=$((RECORDING_MINUTES % 60))
    
    echo "⏱️  Capacité d'enregistrement: ${RECORDING_HOURS}h ${RECORDING_MINUTES_REMAINDER}min"
else
    echo "❌ Impossible de calculer le temps d'enregistrement"
fi

# Éjection finale
echo "⏏️  Éjection..."
sudo eject "$DEVICE"

echo "✅ Terminé!"