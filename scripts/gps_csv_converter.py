#!/usr/bin/env python3
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
Extracteur de données GPS depuis les fichiers MCAP
Exploration récursive des dossiers pour trouver les fichiers MCAP
Version intégrée avec import_recordings.py
"""

import os
import json
import pandas as pd
import numpy as np
from datetime import datetime

# Imports MCAP
try:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import Decoder
    MCAP_AVAILABLE = True
except ImportError:
    MCAP_AVAILABLE = False

# =====================================================================================
# EXCEPTIONS PERSONNALISÉES
# =====================================================================================

class GpsConverterError(Exception):
    """Exception personnalisée pour les erreurs de conversion GPS"""
    pass

# =====================================================================================
# FONCTIONS UTILITAIRES
# =====================================================================================

def demander_dossier_principal():
    """
    Demande à l'utilisateur le chemin complet du dossier principal contenant les données
    """
    print("SELECTION DU DOSSIER DE DONNEES GPS")
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

def analyser_topics_gps_dans_fichier(mcap_path):
    """
    Analyse un fichier MCAP pour trouver les topics GPS
    """
    topics_gps_possibles = [
        '/gps/fix',
        '/navsat/fix', 
        '/gps/navsat',
        '/fix',
        '/gnss/fix',
        '/ublox_gps/fix',
        '/gps_driver/fix'
    ]
    
    topics_gps_trouves = []
    
    try:
        with open(mcap_path, 'rb') as f:
            reader = make_reader(f)
            
            # Lister tous les topics
            available_topics = []
            for schema, channel, message in reader.iter_messages():
                if channel.topic not in available_topics:
                    available_topics.append(channel.topic)
            
            # Chercher les topics GPS
            for topic in available_topics:
                if any(gps_keyword in topic.lower() for gps_keyword in ['gps', 'navsat', 'gnss', 'fix']):
                    if topic not in topics_gps_trouves:
                        topics_gps_trouves.append(topic)
    
    except Exception as e:
        print(f"      Erreur analyse {os.path.basename(mcap_path)}: {e}")
    
    return topics_gps_trouves

def choisir_format_sortie():
    """
    Choix du format de sortie pour les données GPS
    """
    print("CHOIX DU FORMAT DE SORTIE GPS")
    print("=" * 40)
    print("1. JSON - Format structuré")
    print("2. CSV - Compatible Excel/analyse")
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

# =====================================================================================
# FONCTIONS DE CONVERSION
# =====================================================================================

def extraire_donnees_message_gps(gps_msg, message, schema):
    """
    Extrait les données d'un message GPS
    """
    try:
        timestamp_sec = message.log_time / 1e9
        datetime_str = datetime.fromtimestamp(timestamp_sec).isoformat()
        
        # Structure de base
        gps_entry = {
            'timestamp_sec': timestamp_sec,
            'datetime': datetime_str,
            'message_type': schema.name
        }
        
        # Données communes NavSatFix
        if hasattr(gps_msg, 'latitude'):
            gps_entry.update({
                'latitude': float(gps_msg.latitude),
                'longitude': float(gps_msg.longitude),
                'altitude': float(gps_msg.altitude),
                'status': int(gps_msg.status.status) if hasattr(gps_msg, 'status') else None,
                'service': int(gps_msg.status.service) if hasattr(gps_msg, 'status') else None
            })
            
            # Covariance de position
            if hasattr(gps_msg, 'position_covariance'):
                # Convertir la matrice de covariance en liste pour JSON/CSV
                covariance_list = list(gps_msg.position_covariance)
                gps_entry['position_covariance'] = covariance_list
                gps_entry['covariance_type'] = int(gps_msg.position_covariance_type)
                
                # Extraire les variances principales (éléments diagonaux)
                if len(covariance_list) >= 9:
                    gps_entry['variance_east'] = covariance_list[0]      # σ²_x
                    gps_entry['variance_north'] = covariance_list[4]     # σ²_y  
                    gps_entry['variance_up'] = covariance_list[8]        # σ²_z
        
        # Données étendues (si disponibles)
        if hasattr(gps_msg, 'header'):
            gps_entry['frame_id'] = gps_msg.header.frame_id
            gps_entry['header_timestamp_sec'] = (
                gps_msg.header.stamp.sec + gps_msg.header.stamp.nanosec / 1e9
            )
        
        return gps_entry
        
    except Exception as e:
        print(f"        Erreur extraction GPS: {e}")
        return None

def convertir_gps_vers_json(mcap_path, topics_gps, output_dir, mcap_name_base):
    """
    Convertit les données GPS d'un fichier MCAP en JSON - DIRECTEMENT dans output_dir
    """
    print(f"   Conversion JSON GPS de {os.path.basename(mcap_path)}...")
    
    gps_data = {}
    
    try:
        with open(mcap_path, 'rb') as f:
            reader = make_reader(f)
            decoder = Decoder()
            
            # Initialiser la structure pour chaque topic GPS
            for topic in topics_gps:
                gps_data[topic] = {
                    "metadata": {
                        "topic_name": topic,
                        "mcap_file": os.path.basename(mcap_path),
                        "mcap_path": mcap_path,
                        "extraction_time": datetime.now().isoformat(),
                        "total_points": 0
                    },
                    "gps_points": []
                }
            
            message_count = 0
            for schema, channel, message in reader.iter_messages():
                if channel.topic in topics_gps:
                    try:
                        # Décoder le message GPS
                        gps_msg = decoder.decode(schema, message)
                        
                        # Extraire les données GPS
                        gps_entry = extraire_donnees_message_gps(gps_msg, message, schema)
                        
                        if gps_entry:
                            gps_data[channel.topic]["gps_points"].append(gps_entry)
                            gps_data[channel.topic]["metadata"]["total_points"] += 1
                            message_count += 1
                            
                            if message_count % 100 == 0:
                                print(f"      Messages GPS traités: {message_count}")
                    
                    except Exception as e:
                        print(f"      Erreur message GPS: {e}")
                        continue
            
            print(f"      Total messages GPS: {message_count}")
    
    except Exception as e:
        print(f"      Erreur lecture {os.path.basename(mcap_path)}: {e}")
        return 0
    
    # Sauvegarder les fichiers JSON directement dans output_dir
    fichiers_crees = 0
    for topic, data in gps_data.items():
        if data["gps_points"]:
            topic_clean = topic.replace('/', '_').replace(':', '_')
            json_filename = f"{mcap_name_base}_{topic_clean}_gps.json"
            json_path = os.path.join(output_dir, json_filename)
            
            with open(json_path, 'w') as f:
                json.dump(data, f, indent=2)
            
            print(f"      {len(data['gps_points'])} points GPS sauvegardés dans {json_filename}")
            fichiers_crees += 1
    
    return fichiers_crees

def convertir_gps_vers_csv(mcap_path, topics_gps, output_dir, mcap_name_base):
    """
    Convertit les données GPS d'un fichier MCAP en CSV - DIRECTEMENT dans output_dir
    """
    print(f"   Conversion CSV GPS de {os.path.basename(mcap_path)}...")
    
    gps_data = {topic: [] for topic in topics_gps}
    
    try:
        with open(mcap_path, 'rb') as f:
            reader = make_reader(f)
            decoder = Decoder()
            
            message_count = 0
            for schema, channel, message in reader.iter_messages():
                if channel.topic in topics_gps:
                    try:
                        # Décoder le message GPS
                        gps_msg = decoder.decode(schema, message)
                        
                        # Extraire les données GPS
                        gps_entry = extraire_donnees_message_gps(gps_msg, message, schema)
                        
                        if gps_entry:
                            gps_data[channel.topic].append(gps_entry)
                            message_count += 1
                            
                            if message_count % 100 == 0:
                                print(f"      Messages GPS traités: {message_count}")
                    
                    except Exception as e:
                        print(f"      Erreur message GPS: {e}")
                        continue
            
            print(f"      Total messages GPS: {message_count}")
    
    except Exception as e:
        print(f"      Erreur lecture {os.path.basename(mcap_path)}: {e}")
        return 0
    
    # Sauvegarder les fichiers CSV directement dans output_dir
    fichiers_crees = 0
    for topic, data in gps_data.items():
        if data:
            topic_clean = topic.replace('/', '_').replace(':', '_')
            csv_filename = f"{mcap_name_base}_{topic_clean}_gps.csv"
            csv_path = os.path.join(output_dir, csv_filename)
            
            df = pd.DataFrame(data)
            df.to_csv(csv_path, index=False)
            
            print(f"      {len(data)} points GPS sauvegardés dans {csv_filename}")
            
            # Afficher des statistiques GPS
            if 'latitude' in df.columns and 'longitude' in df.columns:
                print(f"        Latitude: {df['latitude'].min():.6f} à {df['latitude'].max():.6f}")
                print(f"        Longitude: {df['longitude'].min():.6f} à {df['longitude'].max():.6f}")
                if 'altitude' in df.columns:
                    print(f"        Altitude: {df['altitude'].min():.1f} à {df['altitude'].max():.1f} m")
            
            fichiers_crees += 1
    
    return fichiers_crees

# =====================================================================================
# TRAITEMENT EN LOT
# =====================================================================================

def traiter_dossiers_out_gps(dossiers_out, format_choisi):
    """
    Traite tous les dossiers 'out' trouvés pour extraire les données GPS
    """
    print(f"\nDEBUT DE L'EXTRACTION GPS")
    print("=" * 40)
    
    # Statistiques globales
    total_fichiers = 0
    conversions_reussies = 0
    
    # Traiter chaque dossier "out" trouvé
    for i, dossier_info in enumerate(dossiers_out, 1):
        chemin_out = dossier_info['chemin']
        chemin_relatif = dossier_info['chemin_relatif']
        fichiers_mcap = dossier_info['fichiers_mcap']
        
        print(f"\n[{i}/{len(dossiers_out)}] Traitement GPS de: {chemin_relatif}")
        print("-" * 50)
        
        # Créer un seul dossier de sortie à côté du dossier "out"
        dossier_parent = os.path.dirname(chemin_out)
        output_dir = os.path.join(dossier_parent, "gps_conversions")
        os.makedirs(output_dir, exist_ok=True)
        
        # Traiter chaque fichier MCAP dans ce dossier
        for mcap_file in fichiers_mcap:
            mcap_path = os.path.join(chemin_out, mcap_file)
            mcap_name_base = mcap_file.replace('.mcap', '')
            
            # Analyser les topics GPS dans ce fichier
            topics_gps = analyser_topics_gps_dans_fichier(mcap_path)
            
            if not topics_gps:
                print(f"   Aucun topic GPS trouvé dans {mcap_file}")
                total_fichiers += 1
                continue
            
            print(f"   Topics GPS trouvés dans {mcap_file}: {topics_gps}")
            
            try:
                fichiers_crees = 0
                
                # Convertir selon le format choisi
                if format_choisi in ["json", "both"]:
                    fichiers_crees += convertir_gps_vers_json(mcap_path, topics_gps, output_dir, mcap_name_base)
                
                if format_choisi in ["csv", "both"]:
                    fichiers_crees += convertir_gps_vers_csv(mcap_path, topics_gps, output_dir, mcap_name_base)
                
                if fichiers_crees > 0:
                    conversions_reussies += 1
                
                total_fichiers += 1
                
            except Exception as e:
                print(f"   ERREUR: {e}")
                total_fichiers += 1
    
    # Résumé final
    print(f"\nRESUME DE L'EXTRACTION GPS")
    print("=" * 40)
    print(f"Fichiers traités: {total_fichiers}")
    print(f"Extractions réussies: {conversions_reussies}")
    print(f"Échecs: {total_fichiers - conversions_reussies}")
    print(f"Taux de réussite: {(conversions_reussies/total_fichiers*100):.1f}%" if total_fichiers > 0 else "0%")
    print(f"Fichiers de sortie dans les dossiers 'gps_conversions'")

# =====================================================================================
# API POUR INTEGRATION AVEC import_recordings.py
# =====================================================================================

def convert_gps_batch(root_directory, output_format="both", custom_topics=None, verbose=True):
    """
    API pour convertir les données GPS en mode batch (appelée par import_recordings.py)
    
    Args:
        root_directory (str): Répertoire racine contenant le dossier 'out' avec les MCAP
        output_format (str): Format de sortie ('json', 'csv', 'both')
        custom_topics (list): Topics GPS personnalisés à rechercher (optionnel)
        verbose (bool): Affichage détaillé
    
    Returns:
        dict: Statistiques d'extraction
    """
    try:
        # Chercher le dossier 'out' dans le répertoire racine
        out_dir = os.path.join(root_directory, "out")
        if not os.path.exists(out_dir):
            if verbose:
                print(f"   Dossier 'out' non trouvé dans {root_directory}")
            return {
                'total_fichiers': 0,
                'extractions_reussies': 0,
                'fichiers_crees': 0,
                'erreurs': ['Dossier out non trouvé']
            }
        
        # Créer le dossier de sortie
        output_dir = os.path.join(root_directory, "gps_conversions")
        os.makedirs(output_dir, exist_ok=True)
        
        # Chercher les fichiers MCAP
        fichiers_mcap = [f for f in os.listdir(out_dir) if f.endswith('.mcap')]
        
        if not fichiers_mcap:
            if verbose:
                print(f"   Aucun fichier MCAP trouvé dans {out_dir}")
            return {
                'total_fichiers': 0,
                'extractions_reussies': 0,
                'fichiers_crees': 0,
                'erreurs': ['Aucun fichier MCAP trouvé']
            }
        
        total_fichiers = len(fichiers_mcap)
        extractions_reussies = 0
        fichiers_crees_total = 0
        erreurs = []
        
        if verbose:
            print(f"   Traitement de {total_fichiers} fichiers MCAP pour GPS...")
        
        # Traiter chaque fichier MCAP
        for mcap_file in fichiers_mcap:
            mcap_path = os.path.join(out_dir, mcap_file)
            mcap_name_base = mcap_file.replace('.mcap', '')
            
            try:
                # Analyser les topics GPS dans ce fichier
                if custom_topics:
                    # Utiliser les topics personnalisés fournis
                    topics_gps = custom_topics
                    if verbose:
                        print(f"     Utilisation des topics personnalisés: {topics_gps}")
                else:
                    # Découvrir automatiquement les topics GPS
                    topics_gps = analyser_topics_gps_dans_fichier(mcap_path)
                
                if not topics_gps:
                    if verbose:
                        print(f"     ⚠ {mcap_file}: aucun topic GPS trouvé")
                    continue
                
                if verbose:
                    print(f"     Topics GPS dans {mcap_file}: {topics_gps}")
                
                fichiers_crees = 0
                
                # Convertir selon le format demandé
                if output_format in ["json", "both"]:
                    fichiers_crees += convertir_gps_vers_json(mcap_path, topics_gps, output_dir, mcap_name_base)
                
                if output_format in ["csv", "both"]:
                    fichiers_crees += convertir_gps_vers_csv(mcap_path, topics_gps, output_dir, mcap_name_base)
                
                if fichiers_crees > 0:
                    extractions_reussies += 1
                    fichiers_crees_total += fichiers_crees
                    if verbose:
                        print(f"     ✓ {mcap_file}: {fichiers_crees} fichiers créés")
                else:
                    if verbose:
                        print(f"     ⚠ {mcap_file}: aucune donnée GPS extraite")
                        
            except Exception as e:
                error_msg = f"Erreur {mcap_file}: {str(e)}"
                erreurs.append(error_msg)
                if verbose:
                    print(f"     ✗ {error_msg}")
        
        return {
            'total_fichiers': total_fichiers,
            'extractions_reussies': extractions_reussies,
            'fichiers_crees': fichiers_crees_total,
            'erreurs': erreurs
        }
        
    except Exception as e:
        raise GpsConverterError(f"Erreur dans convert_gps_batch: {str(e)}")

# =====================================================================================
# FONCTION PRINCIPALE
# =====================================================================================

def main():
    """
    Fonction principale pour l'extraction GPS
    """
    print("EXTRACTEUR DE DONNEES GPS - EXPLORATION RECURSIVE")
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
    
    # Demander confirmation avant extraction
    print(f"\nVoulez-vous extraire les données GPS de tous ces fichiers MCAP ? (o/n): ", end="")
    confirmation = input().strip().lower()
    
    if confirmation in ['o', 'oui', 'y', 'yes']:
        # Lancer l'extraction
        traiter_dossiers_out_gps(dossiers_out, format_choisi)
    else:
        print("Extraction annulée.")

if __name__ == "__main__":
    main()