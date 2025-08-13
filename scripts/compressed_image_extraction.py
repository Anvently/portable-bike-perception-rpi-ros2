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
Extracteur d'images compressées depuis les fichiers MCAP
Exploration récursive des dossiers pour trouver les fichiers MCAP
Version intégrée avec import_recordings.py
"""

#!/usr/bin/env python3
"""
Extracteur d'images compressées depuis les fichiers MCAP
Exploration récursive des dossiers pour trouver les fichiers MCAP
Version intégrée avec import_recordings.py
"""

import os
import json
import pandas as pd
import numpy as np
from datetime import datetime
import base64

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

class ImageConverterError(Exception):
    """Exception personnalisée pour les erreurs de conversion d'images"""
    pass

# =====================================================================================
# FONCTIONS UTILITAIRES
# =====================================================================================

def demander_dossier_principal():
    """
    Demande à l'utilisateur le nom du dossier principal contenant les données
    """
    print("SELECTION DU DOSSIER DE DONNEES CAMERA")
    print("=" * 40)
    
    while True:
        dossier_nom = input("Entrez le nom du dossier contenant vos données: ").strip()
        
        if not dossier_nom:
            print("Nom de dossier vide. Veuillez entrer un nom valide.")
            continue
        
        # Vérifier si le dossier existe
        if os.path.exists(dossier_nom) and os.path.isdir(dossier_nom):
            print(f"Dossier trouvé: {os.path.abspath(dossier_nom)}")
            return os.path.abspath(dossier_nom)
        else:
            print(f"ERREUR: Le dossier '{dossier_nom}' n'existe pas.")
            print("Vérifiez le nom et réessayez.")

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

def analyser_topics_camera_dans_fichier(mcap_path):
    """
    Analyse un fichier MCAP pour trouver les topics d'images compressées
    """
    topics_camera_possibles = [
        '/images/compressed',
        '/camera/image_raw/compressed',
        '/camera/compressed',
        '/image_compressed',
        '/usb_cam/image_raw/compressed',
        '/front_camera/compressed'
    ]
    
    topics_camera_trouves = []
    
    try:
        with open(mcap_path, 'rb') as f:
            reader = make_reader(f)
            
            # Lister tous les topics
            available_topics = []
            topic_types = {}
            for schema, channel, message in reader.iter_messages():
                if channel.topic not in available_topics:
                    available_topics.append(channel.topic)
                    topic_types[channel.topic] = schema.name
            
            # Chercher les topics d'images compressées
            for topic in available_topics:
                # Vérifier le type de message ET le nom du topic
                if (topic_types.get(topic) == 'sensor_msgs/msg/CompressedImage' or
                    any(keyword in topic.lower() for keyword in ['compressed', 'image']) and 
                    'compressed' in topic.lower()):
                    if topic not in topics_camera_trouves:
                        topics_camera_trouves.append(topic)
    
    except Exception as e:
        print(f"      Erreur analyse {os.path.basename(mcap_path)}: {e}")
    
    return topics_camera_trouves

def choisir_format_extraction():
    """
    Choix du format d'extraction pour les images
    """
    print("CHOIX DU FORMAT D'EXTRACTION IMAGES")
    print("=" * 40)
    print("1. Images JPEG séparées - Un fichier .jpg par image")
    print("2. CSV avec métadonnées - Infos images sans données binaires")
    print("3. JSON avec images base64 - Images encodées dans JSON")
    print("4. Tous les formats")
    print()
    
    while True:
        choix = input("Votre choix (1, 2, 3, ou 4): ").strip()
        if choix == "1":
            return "images"
        elif choix == "2":
            return "csv"
        elif choix == "3":
            return "json"
        elif choix == "4":
            return "all"
        else:
            print("Choix invalide. Tapez 1, 2, 3, ou 4.")

# =====================================================================================
# FONCTIONS DE CONVERSION
# =====================================================================================

def extraire_donnees_message_image(image_msg, message, schema):
    """
    Extrait les données d'un message d'image compressée
    """
    try:
        timestamp_sec = message.log_time / 1e9
        datetime_str = datetime.fromtimestamp(timestamp_sec).isoformat()
        
        # Données de base
        image_entry = {
            'timestamp_sec': timestamp_sec,
            'datetime': datetime_str,
            'message_type': schema.name,
            'format': image_msg.format if hasattr(image_msg, 'format') else 'jpeg',
            'data_size': len(image_msg.data) if hasattr(image_msg, 'data') else 0
        }
        
        # Header information
        if hasattr(image_msg, 'header'):
            image_entry.update({
                'frame_id': image_msg.header.frame_id,
                'header_timestamp_sec': (
                    image_msg.header.stamp.sec + image_msg.header.stamp.nanosec / 1e9
                )
            })
        
        # Données binaires de l'image (pour sauvegarde)
        if hasattr(image_msg, 'data'):
            image_entry['image_data'] = bytes(image_msg.data)
        
        return image_entry
        
    except Exception as e:
        print(f"        Erreur extraction image: {e}")
        return None

def extraire_images_jpeg(mcap_path, topics_camera, output_dir, mcap_name_base):
    """
    Extrait les images JPEG et les sauvegarde comme fichiers séparés
    """
    images_dir = os.path.join(output_dir, "images")
    os.makedirs(images_dir, exist_ok=True)
    
    print(f"   Extraction images JPEG de {os.path.basename(mcap_path)}...")
    
    try:
        with open(mcap_path, 'rb') as f:
            reader = make_reader(f)
            decoder = Decoder()
            
            image_count = 0
            for schema, channel, message in reader.iter_messages():
                if channel.topic in topics_camera:
                    try:
                        # Décoder le message d'image
                        image_msg = decoder.decode(schema, message)
                        
                        # Extraire les données d'image
                        image_entry = extraire_donnees_message_image(image_msg, message, schema)
                        
                        if image_entry and 'image_data' in image_entry:
                            # Créer le nom de fichier avec date_heure et timestamp Unix
                            timestamp_sec = image_entry['timestamp_sec']
                            
                            # Format date_heure : YYYY-MM-DD_HH-MM-SS
                            date_heure = datetime.fromtimestamp(timestamp_sec).strftime('%Y-%m-%d_%H-%M-%S')
                            
                            # Timestamp Unix (secondes avec 3 décimales pour les millisecondes)
                            timestamp_unix = f"{timestamp_sec:.3f}"
                            
                            # Topic nettoyé
                            topic_clean = channel.topic.replace('/', '_').replace(':', '_')
                            
                            # Nom de fichier : mcap_topic_date-heure_timestamp-unix_index.jpg
                            image_filename = f"{mcap_name_base}_{topic_clean}_{date_heure}_{timestamp_unix}_{image_count:06d}.jpg"
                            image_path = os.path.join(images_dir, image_filename)
                            
                            # Sauvegarder l'image JPEG
                            with open(image_path, 'wb') as img_file:
                                img_file.write(image_entry['image_data'])
                            
                            image_count += 1
                            
                            if image_count % 10 == 0:
                                print(f"      Images extraites: {image_count}")
                    
                    except Exception as e:
                        print(f"      Erreur extraction image: {e}")
                        continue
            
            print(f"      Total images extraites: {image_count}")
            return image_count
    
    except Exception as e:
        print(f"      Erreur lecture {os.path.basename(mcap_path)}: {e}")
        return 0

def convertir_images_vers_csv(mcap_path, topics_camera, output_dir, mcap_name_base):
    """
    Convertit les métadonnées des images en CSV (sans données binaires)
    """
    print(f"   Conversion CSV métadonnées images de {os.path.basename(mcap_path)}...")
    
    image_data = {topic: [] for topic in topics_camera}
    
    try:
        with open(mcap_path, 'rb') as f:
            reader = make_reader(f)
            decoder = Decoder()
            
            image_count = 0
            for schema, channel, message in reader.iter_messages():
                if channel.topic in topics_camera:
                    try:
                        # Décoder le message d'image
                        image_msg = decoder.decode(schema, message)
                        
                        # Extraire les métadonnées (sans données binaires)
                        image_entry = extraire_donnees_message_image(image_msg, message, schema)
                        
                        if image_entry:
                            # Préparer les informations demandées
                            timestamp_sec = image_entry['timestamp_sec']
                            
                            # Générer le nom de fichier (même logique que pour l'extraction JPEG)
                            date_heure_filename = datetime.fromtimestamp(timestamp_sec).strftime('%Y-%m-%d_%H-%M-%S')
                            timestamp_unix = f"{timestamp_sec:.3f}"
                            topic_clean = channel.topic.replace('/', '_').replace(':', '_')
                            nom_fichier = f"{mcap_name_base}_{topic_clean}_{date_heure_filename}_{timestamp_unix}_{image_count:06d}.jpg"
                            
                            # CSV simplifié avec seulement 3 colonnes demandées
                            csv_entry = {
                                'nom': nom_fichier,
                                'date_heure': datetime.fromtimestamp(timestamp_sec).strftime('%Y-%m-%d %H:%M:%S'),
                                'timestamp_unix': timestamp_unix
                            }
                            
                            image_data[channel.topic].append(csv_entry)
                            image_count += 1
                            
                            if image_count % 10 == 0:
                                print(f"      Métadonnées traitées: {image_count}")
                    
                    except Exception as e:
                        print(f"      Erreur métadonnées image: {e}")
                        continue
            
            print(f"      Total métadonnées: {image_count}")
    
    except Exception as e:
        print(f"      Erreur lecture {os.path.basename(mcap_path)}: {e}")
        return 0
    
    # Sauvegarder les fichiers CSV
    fichiers_crees = 0
    for topic, data in image_data.items():
        if data:
            topic_clean = topic.replace('/', '_').replace(':', '_')
            csv_filename = f"{mcap_name_base}_{topic_clean}_images_metadata.csv"
            csv_path = os.path.join(output_dir, csv_filename)
            
            df = pd.DataFrame(data)
            df.to_csv(csv_path, index=False)
            
            print(f"      {len(data)} métadonnées sauvegardées dans {csv_filename}")
            
            # Statistiques sur les images
            if len(data) > 0:
                duree = float(data[-1]['timestamp_unix']) - float(data[0]['timestamp_unix'])
                fps = len(data) / duree if duree > 0 else 0
                print(f"        Durée: {duree:.1f}s, FPS moyen: {fps:.1f}, {len(data)} images")
            
            fichiers_crees += 1
    
    return fichiers_crees

def convertir_images_vers_json(mcap_path, topics_camera, output_dir, mcap_name_base):
    """
    Convertit les images en JSON avec encodage base64
    """
    print(f"   Conversion JSON images (base64) de {os.path.basename(mcap_path)}...")
    
    image_data = {}
    
    try:
        with open(mcap_path, 'rb') as f:
            reader = make_reader(f)
            decoder = Decoder()
            
            # Initialiser la structure pour chaque topic
            for topic in topics_camera:
                image_data[topic] = {
                    "metadata": {
                        "topic_name": topic,
                        "mcap_file": os.path.basename(mcap_path),
                        "mcap_path": mcap_path,
                        "extraction_time": datetime.now().isoformat(),
                        "total_images": 0
                    },
                    "images": []
                }
            
            image_count = 0
            for schema, channel, message in reader.iter_messages():
                if channel.topic in topics_camera:
                    try:
                        # Décoder le message d'image
                        image_msg = decoder.decode(schema, message)
                        
                        # Extraire les données d'image
                        image_entry = extraire_donnees_message_image(image_msg, message, schema)
                        
                        if image_entry and 'image_data' in image_entry:
                            # Encoder l'image en base64 pour JSON
                            image_base64 = base64.b64encode(image_entry['image_data']).decode('utf-8')
                            
                            # Préparer l'entrée JSON
                            json_entry = {
                                "image_index": image_count,
                                "timestamp": {
                                    "mcap_time_sec": image_entry['timestamp_sec'],
                                    "datetime_iso": image_entry['datetime']
                                },
                                "metadata": {
                                    "format": image_entry['format'],
                                    "data_size": image_entry['data_size'],
                                    "frame_id": image_entry.get('frame_id', ''),
                                },
                                "image_base64": image_base64
                            }
                            
                            image_data[channel.topic]["images"].append(json_entry)
                            image_data[channel.topic]["metadata"]["total_images"] += 1
                            image_count += 1
                            
                            if image_count % 10 == 0:
                                print(f"      Images encodées: {image_count}")
                    
                    except Exception as e:
                        print(f"      Erreur encodage image: {e}")
                        continue
            
            print(f"      Total images encodées: {image_count}")
    
    except Exception as e:
        print(f"      Erreur lecture {os.path.basename(mcap_path)}: {e}")
        return 0
    
    # Sauvegarder les fichiers JSON
    fichiers_crees = 0
    for topic, data in image_data.items():
        if data["images"]:
            topic_clean = topic.replace('/', '_').replace(':', '_')
            json_filename = f"{mcap_name_base}_{topic_clean}_images.json"
            json_path = os.path.join(output_dir, json_filename)
            
            with open(json_path, 'w') as f:
                json.dump(data, f, indent=2)
            
            print(f"      {len(data['images'])} images sauvegardées dans {json_filename}")
            fichiers_crees += 1
    
    return fichiers_crees

# =====================================================================================
# TRAITEMENT EN LOT
# =====================================================================================

def traiter_dossiers_out_images(dossiers_out, format_choisi):
    """
    Traite tous les dossiers 'out' trouvés pour extraire les images
    """
    print(f"\nDEBUT DE L'EXTRACTION D'IMAGES")
    print("=" * 40)
    
    # Statistiques globales
    total_fichiers = 0
    extractions_reussies = 0
    total_images = 0
    
    # Traiter chaque dossier "out" trouvé
    for i, dossier_info in enumerate(dossiers_out, 1):
        chemin_out = dossier_info['chemin']
        chemin_relatif = dossier_info['chemin_relatif']
        fichiers_mcap = dossier_info['fichiers_mcap']
        
        print(f"\n[{i}/{len(dossiers_out)}] Extraction images de: {chemin_relatif}")
        print("-" * 50)
        
        # Créer un dossier de sortie à côté du dossier "out"
        dossier_parent = os.path.dirname(chemin_out)
        output_dir = os.path.join(dossier_parent, "camera_conversions")
        os.makedirs(output_dir, exist_ok=True)
        
        # Traiter chaque fichier MCAP dans ce dossier
        for mcap_file in fichiers_mcap:
            mcap_path = os.path.join(chemin_out, mcap_file)
            mcap_name_base = mcap_file.replace('.mcap', '')
            
            # Analyser les topics d'images dans ce fichier
            topics_camera = analyser_topics_camera_dans_fichier(mcap_path)
            
            if not topics_camera:
                print(f"   Aucun topic d'images trouvé dans {mcap_file}")
                total_fichiers += 1
                continue
            
            print(f"   Topics images trouvés dans {mcap_file}: {topics_camera}")
            
            try:
                images_extraites = 0
                
                # Extraire selon le format choisi
                if format_choisi in ["images", "all"]:
                    images_extraites += extraire_images_jpeg(mcap_path, topics_camera, output_dir, mcap_name_base)
                
                if format_choisi in ["csv", "all"]:
                    convertir_images_vers_csv(mcap_path, topics_camera, output_dir, mcap_name_base)
                
                if format_choisi in ["json", "all"]:
                    convertir_images_vers_json(mcap_path, topics_camera, output_dir, mcap_name_base)
                
                if images_extraites > 0 or format_choisi in ["csv", "json"]:
                    extractions_reussies += 1
                    total_images += images_extraites
                
                total_fichiers += 1
                
            except Exception as e:
                print(f"   ERREUR: {e}")
                total_fichiers += 1
    
    # Résumé final
    print(f"\nRESUME DE L'EXTRACTION D'IMAGES")
    print("=" * 40)
    print(f"Fichiers traités: {total_fichiers}")
    print(f"Extractions réussies: {extractions_reussies}")
    print(f"Échecs: {total_fichiers - extractions_reussies}")
    print(f"Total images extraites: {total_images}")
    print(f"Taux de réussite: {(extractions_reussies/total_fichiers*100):.1f}%" if total_fichiers > 0 else "0%")
    print(f"Fichiers de sortie dans les dossiers 'camera_conversions'")

# =====================================================================================
# API POUR INTEGRATION AVEC import_recordings.py
# =====================================================================================

def convert_images_batch(root_directory, output_format="all", custom_topics=None, verbose=True):
    """
    API pour convertir les données d'images en mode batch (appelée par import_recordings.py)
    
    Args:
        root_directory (str): Répertoire racine contenant le dossier 'out' avec les MCAP
        output_format (str): Format de sortie ('images', 'csv', 'json', 'all')
        custom_topics (list): Topics d'images personnalisés à rechercher (optionnel)
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
                'images_extraites': 0,
                'fichiers_crees': 0,
                'erreurs': ['Dossier out non trouvé']
            }
        
        # Créer le dossier de sortie
        output_dir = os.path.join(root_directory, "camera_conversions")
        os.makedirs(output_dir, exist_ok=True)
        
        # Chercher les fichiers MCAP
        fichiers_mcap = [f for f in os.listdir(out_dir) if f.endswith('.mcap')]
        
        if not fichiers_mcap:
            if verbose:
                print(f"   Aucun fichier MCAP trouvé dans {out_dir}")
            return {
                'total_fichiers': 0,
                'extractions_reussies': 0,
                'images_extraites': 0,
                'fichiers_crees': 0,
                'erreurs': ['Aucun fichier MCAP trouvé']
            }
        
        total_fichiers = len(fichiers_mcap)
        extractions_reussies = 0
        images_extraites_total = 0
        fichiers_crees_total = 0
        erreurs = []
        
        if verbose:
            print(f"   Traitement de {total_fichiers} fichiers MCAP pour les images...")
        
        # Traiter chaque fichier MCAP
        for mcap_file in fichiers_mcap:
            mcap_path = os.path.join(out_dir, mcap_file)
            mcap_name_base = mcap_file.replace('.mcap', '')
            
            try:
                # Analyser les topics d'images dans ce fichier
                if custom_topics:
                    # Utiliser les topics personnalisés fournis
                    topics_camera = custom_topics
                    if verbose:
                        print(f"     Utilisation des topics personnalisés: {topics_camera}")
                else:
                    # Découvrir automatiquement les topics d'images
                    topics_camera = analyser_topics_camera_dans_fichier(mcap_path)
                
                if not topics_camera:
                    if verbose:
                        print(f"     ⚠ {mcap_file}: aucun topic d'images trouvé")
                    continue
                
                if verbose:
                    print(f"     Topics images dans {mcap_file}: {topics_camera}")
                
                images_extraites = 0
                fichiers_crees = 0
                
                # Convertir selon le format demandé
                if output_format in ["images", "all"]:
                    images_extraites += extraire_images_jpeg(mcap_path, topics_camera, output_dir, mcap_name_base)
                
                if output_format in ["csv", "all"]:
                    fichiers_crees += convertir_images_vers_csv(mcap_path, topics_camera, output_dir, mcap_name_base)
                
                if output_format in ["json", "all"]:
                    fichiers_crees += convertir_images_vers_json(mcap_path, topics_camera, output_dir, mcap_name_base)
                
                if images_extraites > 0 or fichiers_crees > 0:
                    extractions_reussies += 1
                    images_extraites_total += images_extraites
                    fichiers_crees_total += fichiers_crees
                    if verbose:
                        print(f"     ✓ {mcap_file}: {images_extraites} images, {fichiers_crees} fichiers créés")
                else:
                    if verbose:
                        print(f"     ⚠ {mcap_file}: aucune image extraite")
                        
            except Exception as e:
                error_msg = f"Erreur {mcap_file}: {str(e)}"
                erreurs.append(error_msg)
                if verbose:
                    print(f"     ✗ {error_msg}")
        
        return {
            'total_fichiers': total_fichiers,
            'extractions_reussies': extractions_reussies,
            'images_extraites': images_extraites_total,
            'fichiers_crees': fichiers_crees_total,
            'erreurs': erreurs
        }
        
    except Exception as e:
        raise ImageConverterError(f"Erreur dans convert_images_batch: {str(e)}")

# =====================================================================================
# FONCTION PRINCIPALE
# =====================================================================================

def main():
    """
    Fonction principale pour l'extraction d'images
    """
    print("EXTRACTEUR D'IMAGES CAMERA - EXPLORATION RECURSIVE")
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
    
    # Choisir le format d'extraction
    format_choisi = choisir_format_extraction()
    
    # Demander confirmation avant extraction
    print(f"\nVoulez-vous extraire les images de tous ces fichiers MCAP ? (o/n): ", end="")
    confirmation = input().strip().lower()
    
    if confirmation in ['o', 'oui', 'y', 'yes']:
        # Lancer l'extraction
        traiter_dossiers_out_images(dossiers_out, format_choisi)
    else:
        print("Extraction annulée.")

if __name__ == "__main__":
    main()