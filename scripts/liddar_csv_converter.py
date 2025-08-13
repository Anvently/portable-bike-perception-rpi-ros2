#!/usr/bin/env python3
# Created on Tue Aug 13 2025
# Updated on Tue Aug 13 2025
# Enhanced with MCAP converters integration (LiDAR, GPS, Images)
#
#  This file is part of Cyclosafe
# Copyright (c) 2025 Eric Ta
#
# This software is governed by the CeCILL license under French law and
# abiding by the rules of distribution of free software. You can use,
# modify and/or redistribute the software under the terms of the CeCILL
# license as circulated by CEA, CNRS and INRIA at:
# https://cecill.info/licences/Licence_CeCILL-B_V1-en.html

"""
Convertisseur MCAP vers JSON/CSV pour les données LiDAR 360° (LaserScan)
Exploration récursive des dossiers pour trouver les fichiers MCAP
Version intégrée avec import_recordings.py
"""

import os
import json
import numpy as np
import pandas as pd
from datetime import datetime

# Imports MCAP au niveau global
try:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import Decoder
    MCAP_AVAILABLE = True
except ImportError:
    MCAP_AVAILABLE = False

# =====================================================================================
# EXCEPTIONS PERSONNALISÉES
# =====================================================================================

class LidarConverterError(Exception):
    """Exception personnalisée pour les erreurs de conversion LiDAR"""
    pass

# =====================================================================================
# FONCTIONS UTILITAIRES
# =====================================================================================

def demander_dossier_principal():
    """
    Demande à l'utilisateur le chemin complet du dossier principal contenant les données
    """
    print("SELECTION DU DOSSIER DE DONNEES")
    print("=" * 40)
    
    while True:
        dossier_chemin = input("Entrez le chemin complet du dossier contenant vos données: ").strip()
        
        if not dossier_chemin:
            print("Chemin de dossier vide. Veuillez entrer un chemin valide.")
            continue
        
        # Nettoyer le chemin (supprimer guillemets, espaces)
        dossier_chemin = dossier_chemin.strip('"').strip("'").strip()
        
        # Convertir en chemin absolu
        dossier_absolu = os.path.abspath(dossier_chemin)
        
        # Vérifier si le dossier existe
        if os.path.exists(dossier_absolu) and os.path.isdir(dossier_absolu):
            print(f"Dossier trouvé: {dossier_absolu}")
            return dossier_absolu
        else:
            print(f"ERREUR: Le dossier '{dossier_absolu}' n'existe pas.")
            print("Vérifiez le chemin et réessayez.")
            print("Exemples de chemins valides:")
            print("  /home/user/mes_donnees")
            print("  ~/Documents/enregistrements") 
            print("  /media/user/USB_DRIVE/data")

def explorer_dossiers_mcap(dossier_principal):
    """
    Explore récursivement tous les sous-dossiers pour trouver les dossiers 'out' 
    contenant des fichiers MCAP
    """
    print(f"\nEXPLORATION DE {dossier_principal}")
    print("=" * 60)
    
    dossiers_out_trouves = []
    total_fichiers_mcap = 0
    
    # Parcourir récursivement tous les dossiers
    for racine, dossiers, fichiers in os.walk(dossier_principal):
        # Vérifier si le dossier actuel s'appelle "out"
        if os.path.basename(racine) == "out":
            # Chercher les fichiers MCAP dans ce dossier "out"
            fichiers_mcap = [f for f in fichiers if f.endswith('.mcap')]
            
            if fichiers_mcap:
                chemin_relatif = os.path.relpath(racine, dossier_principal)
                dossiers_out_trouves.append({
                    'chemin': racine,
                    'chemin_relatif': chemin_relatif,
                    'fichiers_mcap': fichiers_mcap
                })
                total_fichiers_mcap += len(fichiers_mcap)
                print(f"Trouvé: {chemin_relatif} ({len(fichiers_mcap)} fichiers MCAP)")
    
    print(f"\nRESULTAT DE L'EXPLORATION:")
    print(f"  Dossiers 'out' trouvés: {len(dossiers_out_trouves)}")
    print(f"  Total fichiers MCAP: {total_fichiers_mcap}")
    
    return dossiers_out_trouves

def choisir_format_sortie():
    """
    Demande à l'utilisateur de choisir le format de sortie
    """
    print("CHOIX DU FORMAT DE SORTIE")
    print("=" * 40)
    print("1. JSON - Format structuré, facile à lire")
    print("2. CSV - Format tabulaire, compatible Excel")
    print("3. Les deux formats")
    print()
    
    while True:
        choix = input("Votre choix (1, 2, ou 3): ").strip()
        if choix == "1":
            return "json"
        elif choix == "2":
            return "csv"
        elif choix == "3":
            return "both"
        else:
            print("Choix invalide. Tapez 1, 2, ou 3.")

def convertir_tableaux_pour_csv(array_data):
    """
    Convertit un tableau numpy en string pour stockage CSV
    """
    if len(array_data) == 0:
        return "[]"
    
    # Convertir les valeurs spéciales
    cleaned_data = []
    for value in array_data:
        if np.isfinite(value):
            cleaned_data.append(f"{value:.6f}")
        elif np.isinf(value):
            cleaned_data.append("inf")
        elif np.isnan(value):
            cleaned_data.append("nan")
        else:
            cleaned_data.append("unknown")
    
    # Créer une représentation compacte
    return "[" + ",".join(cleaned_data) + "]"

# =====================================================================================
# FONCTIONS DE CONVERSION
# =====================================================================================

def convertir_mcap_vers_json(mcap_path, output_dir, mcap_name_base):
    """
    Convertit un fichier MCAP en format JSON - DIRECTEMENT dans output_dir
    """
    lidar360_topics = ['/lidar360_1/scan', '/lidar360_2/scan']
    
    print(f"   Conversion JSON de {os.path.basename(mcap_path)}...")
    
    json_data = {}
    
    try:
        with open(mcap_path, 'rb') as f:
            reader = make_reader(f)
            decoder = Decoder()
            
            # Identifier les topics
            available_topics = []
            for schema, channel, message in reader.iter_messages():
                if channel.topic not in available_topics:
                    available_topics.append(channel.topic)
            
            found_topics = [topic for topic in lidar360_topics if topic in available_topics]
            
            if not found_topics:
                print(f"     Aucun topic LiDAR 360° trouvé")
                return 0
            
            # Initialiser structure JSON
            for topic in found_topics:
                json_data[topic] = {
                    "metadata": {
                        "topic_name": topic,
                        "message_type": "sensor_msgs/msg/LaserScan",
                        "mcap_file": os.path.basename(mcap_path),
                        "mcap_path": mcap_path,
                        "extraction_time": datetime.now().isoformat(),
                        "total_scans": 0
                    },
                    "scans": []
                }
            
            # Lire les messages
            with open(mcap_path, 'rb') as f2:
                reader2 = make_reader(f2)
                
                for schema, channel, message in reader2.iter_messages():
                    if channel.topic in found_topics:
                        try:
                            ros_msg = decoder.decode(schema, message)
                            
                            ranges_array = np.array(ros_msg.ranges)
                            intensities_array = np.array(ros_msg.intensities) if ros_msg.intensities else np.array([])
                            
                            # Convertir pour JSON
                            ranges_clean = []
                            for r in ranges_array:
                                if np.isfinite(r):
                                    ranges_clean.append(float(r))
                                elif np.isinf(r):
                                    ranges_clean.append("inf")
                                else:
                                    ranges_clean.append("nan")
                            
                            intensities_clean = []
                            for i in intensities_array:
                                if np.isfinite(i):
                                    intensities_clean.append(float(i))
                                elif np.isinf(i):
                                    intensities_clean.append("inf")
                                else:
                                    intensities_clean.append("nan")
                            
                            scan_data = {
                                "timestamp": {
                                    "mcap_time_sec": float(message.log_time / 1e9),
                                    "datetime_iso": datetime.fromtimestamp(message.log_time / 1e9).isoformat()
                                },
                                "geometry": {
                                    "angle_min": float(ros_msg.angle_min),
                                    "angle_max": float(ros_msg.angle_max),
                                    "angle_increment": float(ros_msg.angle_increment),
                                    "range_min": float(ros_msg.range_min),
                                    "range_max": float(ros_msg.range_max),
                                    "num_points": len(ranges_array)
                                },
                                "data": {
                                    "ranges": ranges_clean,
                                    "intensities": intensities_clean if len(intensities_clean) > 0 else None
                                },
                                "scan_info": {
                                    "frame_id": ros_msg.header.frame_id,
                                    "scan_time": float(ros_msg.scan_time),
                                    "time_increment": float(ros_msg.time_increment)
                                }
                            }
                            
                            json_data[channel.topic]["scans"].append(scan_data)
                            json_data[channel.topic]["metadata"]["total_scans"] += 1
                            
                        except Exception as e:
                            print(f"     Erreur message: {e}")
                            continue
        
        # Sauvegarder JSON directement dans output_dir
        fichiers_crees = 0
        for topic, data in json_data.items():
            if data["scans"]:
                topic_name = topic.replace('/', '_').replace(':', '_')
                json_filename = f"{mcap_name_base}_{topic_name}_scan.json"
                json_path = os.path.join(output_dir, json_filename)
                
                with open(json_path, 'w') as f:
                    json.dump(data, f, indent=2)
                
                print(f"     {len(data['scans'])} scans sauvegardés dans {json_filename}")
                fichiers_crees += 1
        
        return fichiers_crees
        
    except Exception as e:
        print(f"     Erreur: {e}")
        return 0

def convertir_mcap_vers_csv(mcap_path, output_dir, mcap_name_base):
    """
    Convertit un fichier MCAP en format CSV - DIRECTEMENT dans output_dir
    """
    lidar360_topics = ['/lidar360_1/scan', '/lidar360_2/scan']
    
    print(f"   Conversion CSV de {os.path.basename(mcap_path)}...")
    
    topic_data = {topic: [] for topic in lidar360_topics}
    
    try:
        with open(mcap_path, 'rb') as f:
            reader = make_reader(f)
            decoder = Decoder()
            
            # Identifier les topics
            available_topics = []
            for schema, channel, message in reader.iter_messages():
                if channel.topic not in available_topics:
                    available_topics.append(channel.topic)
            
            found_topics = [topic for topic in lidar360_topics if topic in available_topics]
            
            if not found_topics:
                print(f"     Aucun topic LiDAR 360° trouvé")
                return 0
            
            # Lire les messages
            with open(mcap_path, 'rb') as f2:
                reader2 = make_reader(f2)
                
                for schema, channel, message in reader2.iter_messages():
                    if channel.topic in found_topics:
                        try:
                            ros_msg = decoder.decode(schema, message)
                            
                            ranges_array = np.array(ros_msg.ranges)
                            intensities_array = np.array(ros_msg.intensities) if ros_msg.intensities else np.array([])
                            
                            # Préparer les données CSV
                            row = {
                                # Informations temporelles
                                'timestamp_sec': float(message.log_time / 1e9),
                                'datetime': datetime.fromtimestamp(message.log_time / 1e9).strftime('%Y-%m-%d %H:%M:%S.%f'),
                                
                                # Informations du scan
                                'frame_id': ros_msg.header.frame_id,
                                'scan_time': float(ros_msg.scan_time),
                                'time_increment': float(ros_msg.time_increment),
                                
                                # Géométrie
                                'angle_min': float(ros_msg.angle_min),
                                'angle_max': float(ros_msg.angle_max),
                                'angle_increment': float(ros_msg.angle_increment),
                                'range_min': float(ros_msg.range_min),
                                'range_max': float(ros_msg.range_max),
                                'num_points': len(ranges_array),
                                
                                # TABLEAUX COMPLETS dans une seule cellule
                                'ranges_array': convertir_tableaux_pour_csv(ranges_array),
                                'intensities_array': convertir_tableaux_pour_csv(intensities_array),
                                
                                # Statistiques calculées
                                'range_mean': float(np.nanmean(ranges_array[np.isfinite(ranges_array)])) if np.any(np.isfinite(ranges_array)) else None,
                                'range_std': float(np.nanstd(ranges_array[np.isfinite(ranges_array)])) if np.any(np.isfinite(ranges_array)) else None,
                                'valid_points': int(np.sum(np.isfinite(ranges_array))),
                                'inf_points': int(np.sum(np.isinf(ranges_array))),
                                'nan_points': int(np.sum(np.isnan(ranges_array))),
                                'validity_ratio': float(np.sum(np.isfinite(ranges_array)) / len(ranges_array)) if len(ranges_array) > 0 else 0
                            }
                            
                            topic_data[channel.topic].append(row)
                            
                        except Exception as e:
                            print(f"     Erreur message: {e}")
                            continue
        
        # Sauvegarder CSV directement dans output_dir
        fichiers_crees = 0
        for topic, data in topic_data.items():
            if data:
                topic_name = topic.replace('/', '_').replace(':', '_')
                csv_filename = f"{mcap_name_base}_{topic_name}_scan.csv"
                csv_path = os.path.join(output_dir, csv_filename)
                
                df = pd.DataFrame(data)
                df.to_csv(csv_path, index=False)
                
                print(f"     {len(data)} scans sauvegardés dans {csv_filename}")
                fichiers_crees += 1
        
        return fichiers_crees
        
    except Exception as e:
        print(f"     Erreur: {e}")
        return 0

# =====================================================================================
# TRAITEMENT EN LOT
# =====================================================================================

def traiter_dossiers_out(dossiers_out, format_choisi):
    """
    Traite tous les dossiers 'out' trouvés selon le format choisi
    """
    print(f"\nDEBUT DE LA CONVERSION")
    print("=" * 40)
    
    # Statistiques globales
    total_fichiers = 0
    conversions_reussies = 0
    
    # Traiter chaque dossier "out" trouvé
    for i, dossier_info in enumerate(dossiers_out, 1):
        chemin_out = dossier_info['chemin']
        chemin_relatif = dossier_info['chemin_relatif']
        fichiers_mcap = dossier_info['fichiers_mcap']
        
        print(f"\n[{i}/{len(dossiers_out)}] Traitement de: {chemin_relatif}")
        print("-" * 50)
        
        dossier_parent = os.path.dirname(chemin_out)
        output_dir = os.path.join(dossier_parent, "lidar360_conversions")
        os.makedirs(output_dir, exist_ok=True)
        
        # Traiter chaque fichier MCAP dans ce dossier
        for mcap_file in fichiers_mcap:
            mcap_path = os.path.join(chemin_out, mcap_file)
            mcap_name_base = mcap_file.replace('.mcap', '')
            
            try:
                fichiers_crees = 0
                
                # Convertir selon le format choisi
                if format_choisi in ["json", "both"]:
                    fichiers_crees += convertir_mcap_vers_json(mcap_path, output_dir, mcap_name_base)
                
                if format_choisi in ["csv", "both"]:
                    fichiers_crees += convertir_mcap_vers_csv(mcap_path, output_dir, mcap_name_base)
                
                if fichiers_crees > 0:
                    conversions_reussies += 1
                
                total_fichiers += 1
                
            except Exception as e:
                print(f"   ERREUR: {e}")
                total_fichiers += 1
    
    # Résumé final
    print(f"\nRESUME DE LA CONVERSION")
    print("=" * 40)
    print(f"Fichiers traités: {total_fichiers}")
    print(f"Conversions réussies: {conversions_reussies}")
    print(f"Échecs: {total_fichiers - conversions_reussies}")
    print(f"Taux de réussite: {(conversions_reussies/total_fichiers*100):.1f}%" if total_fichiers > 0 else "0%")
    print(f"Fichiers de sortie dans les dossiers 'lidar360_conversions'")

# =====================================================================================
# API POUR INTEGRATION AVEC import_recordings.py
# =====================================================================================

def convert_lidar_batch(root_directory, output_format="both", verbose=True):
    """
    API pour convertir les données LiDAR en mode batch (appelée par import_recordings.py)
    
    Args:
        root_directory (str): Répertoire racine contenant le dossier 'out' avec les MCAP
        output_format (str): Format de sortie ('json', 'csv', 'both')
        verbose (bool): Affichage détaillé
    
    Returns:
        dict: Statistiques de conversion
    """
    try:
        # Chercher le dossier 'out' dans le répertoire racine
        out_dir = os.path.join(root_directory, "out")
        if not os.path.exists(out_dir):
            if verbose:
                print(f"   Dossier 'out' non trouvé dans {root_directory}")
            return {
                'total_fichiers': 0,
                'conversions_reussies': 0,
                'fichiers_crees': 0,
                'erreurs': ['Dossier out non trouvé']
            }
        
        # Créer le dossier de sortie
        output_dir = os.path.join(root_directory, "lidar360_conversions")
        os.makedirs(output_dir, exist_ok=True)
        
        # Chercher les fichiers MCAP
        fichiers_mcap = [f for f in os.listdir(out_dir) if f.endswith('.mcap')]
        
        if not fichiers_mcap:
            if verbose:
                print(f"   Aucun fichier MCAP trouvé dans {out_dir}")
            return {
                'total_fichiers': 0,
                'conversions_reussies': 0,
                'fichiers_crees': 0,
                'erreurs': ['Aucun fichier MCAP trouvé']
            }
        
        total_fichiers = len(fichiers_mcap)
        conversions_reussies = 0
        fichiers_crees_total = 0
        erreurs = []
        
        if verbose:
            print(f"   Traitement de {total_fichiers} fichiers MCAP pour LiDAR...")
        
        # Traiter chaque fichier MCAP
        for mcap_file in fichiers_mcap:
            mcap_path = os.path.join(out_dir, mcap_file)
            mcap_name_base = mcap_file.replace('.mcap', '')
            
            try:
                fichiers_crees = 0
                
                # Convertir selon le format demandé
                if output_format in ["json", "both"]:
                    fichiers_crees += convertir_mcap_vers_json(mcap_path, output_dir, mcap_name_base)
                
                if output_format in ["csv", "both"]:
                    fichiers_crees += convertir_mcap_vers_csv(mcap_path, output_dir, mcap_name_base)
                
                if fichiers_crees > 0:
                    conversions_reussies += 1
                    fichiers_crees_total += fichiers_crees
                    if verbose:
                        print(f"     ✓ {mcap_file}: {fichiers_crees} fichiers créés")
                else:
                    if verbose:
                        print(f"     ⚠ {mcap_file}: aucune donnée LiDAR trouvée")
                        
            except Exception as e:
                error_msg = f"Erreur {mcap_file}: {str(e)}"
                erreurs.append(error_msg)
                if verbose:
                    print(f"     ✗ {error_msg}")
        
        return {
            'total_fichiers': total_fichiers,
            'conversions_reussies': conversions_reussies,
            'fichiers_crees': fichiers_crees_total,
            'erreurs': erreurs
        }
        
    except Exception as e:
        raise LidarConverterError(f"Erreur dans convert_lidar_batch: {str(e)}")

# =====================================================================================
# FONCTION PRINCIPALE
# =====================================================================================

def main():
    """
    Fonction principale
    """
    print("CONVERTISSEUR MCAP LIDAR 360° - EXPLORATION RECURSIVE")
    print("=" * 70)
    
    # Vérifier les dépendances
    if not MCAP_AVAILABLE:
        print("Dépendances manquantes. Installez-les avec:")
        print("pip install mcap mcap-ros2-support pandas")
        return
    
    # Demander le dossier principal
    dossier_principal = demander_dossier_principal()
    
    # Explorer pour trouver les dossiers "out" avec des MCAP
    dossiers_out = explorer_dossiers_mcap(dossier_principal)
    
    if not dossiers_out:
        print("\nAucun dossier 'out' contenant des fichiers MCAP n'a été trouvé.")
        print("Vérifiez la structure de vos dossiers.")
        return
    
    # Choisir le format de sortie
    format_choisi = choisir_format_sortie()
    
    # Demander confirmation avant conversion
    print(f"\nVoulez-vous convertir tous ces fichiers MCAP ? (o/n): ", end="")
    confirmation = input().strip().lower()
    
    if confirmation in ['o', 'oui', 'y', 'yes']:
        # Lancer la conversion
        traiter_dossiers_out(dossiers_out, format_choisi)
    else:
        print("Conversion annulée.")

if __name__ == "__main__":
    main()