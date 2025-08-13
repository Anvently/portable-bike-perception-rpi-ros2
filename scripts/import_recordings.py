#!/usr/bin/env python3
# Created on Tue Aug 05 2025
# Updated on Tue Aug 08 2025
# Enhanced with MCAP converters integration (LiDAR, GPS, Images)
#
#  This file is part of Cyclosafe
# Copyright (c) 2025 Nicolas Pirard @Anvently
#
# This software is governed by the CeCILL license under French law and
# abiding by the rules of distribution of free software. You can use,
# modify and/or redistribute the software under the terms of the CeCILL
# license as circulated by CEA, CNRS and INRIA at:
# https://cecill.info/licences/Licence_CeCILL-B_V1-en.html

import argparse
import os
import subprocess
import sys
import glob
import getpass
import logging
import shutil
from rich.logging import RichHandler

FORMAT = "%(message)s"
logging.basicConfig(
	level="NOTSET", format=FORMAT, datefmt="[%X]", handlers=[RichHandler()]
)  # set level=20 or logging.INFO to turn off debug
logger = logging.getLogger("rich")

# Import des convertisseurs MCAP - MÉTHODE SÉCURISÉE
def import_mcap_converters():
    """Import sécurisé des convertisseurs MCAP"""
    try:
        # Import dynamique pour éviter les imports circulaires
        import importlib
        
        # Import du module LiDAR
        lidar_module = importlib.import_module('liddar_csv_converter')
        if not hasattr(lidar_module, 'convert_lidar_batch'):
            logger.error("Function 'convert_lidar_batch' not found in liddar_csv_converter")
            return None, None, None, None, None, None, None, False
            
        # Import du module GPS
        gps_module = importlib.import_module('gps_csv_converter')
        if not hasattr(gps_module, 'convert_gps_batch'):
            logger.error("Function 'convert_gps_batch' not found in gps_csv_converter")
            return None, None, None, None, None, None, None, False
        
        # Import du module Images
        images_module = importlib.import_module('compressed_image_extraction')
        if not hasattr(images_module, 'convert_images_batch'):
            logger.error("Function 'convert_images_batch' not found in compressed_image_extraction")
            return None, None, None, None, None, None, None, False
        
        # Récupérer les fonctions et classes
        convert_lidar_batch = getattr(lidar_module, 'convert_lidar_batch')
        convert_gps_batch = getattr(gps_module, 'convert_gps_batch')
        convert_images_batch = getattr(images_module, 'convert_images_batch')
        LidarConverterError = getattr(lidar_module, 'LidarConverterError', Exception)
        GpsConverterError = getattr(gps_module, 'GpsConverterError', Exception)
        ImageConverterError = getattr(images_module, 'ImageConverterError', Exception)
        
        logger.info("MCAP converters successfully imported")
        return convert_lidar_batch, convert_gps_batch, convert_images_batch, LidarConverterError, GpsConverterError, ImageConverterError, True
        
    except ImportError as e:
        logger.warning(f"Convertisseurs MCAP non disponibles: {e}")
        return None, None, None, None, None, None, False
    except Exception as e:
        logger.error(f"Erreur lors de l'import des convertisseurs: {e}")
        return None, None, None, None, None, None, False

# Initialisation des convertisseurs
convert_lidar_batch, convert_gps_batch, convert_images_batch, LidarConverterError, GpsConverterError, ImageConverterError, MCAP_CONVERTERS_AVAILABLE = import_mcap_converters()

def parse_arguments():
	"""Parse command line arguments."""
	parser = argparse.ArgumentParser(description='Import, decompress and merge rosbag files from a remote host to single bag file. Convert MCAP data to CSV/JSON/Images. Attempt to repair corrupted bags.')
	parser.add_argument('-u', '--hostname', help='Hostname for SSH connection (user@host-ip)')
	parser.add_argument('-c', '--copy', help='Path to a local source directory (e.g., mounted SD card) to copy from')
	parser.add_argument('-s', '--skip_import', help='Skip import step and use specified local path for conversion')
	parser.add_argument('-o', '--output', help='Output directory for imported bags')
	parser.add_argument('-x', '--clean', action='store_true', help='Delete bag/ directories locally after successful import and conversion')
	

	parser.add_argument('--skip-mcap', action='store_true', help='Skip MCAP conversion (LiDAR, GPS, and Images)')
	parser.add_argument('--mcap-format', choices=['json', 'csv', 'both'], default='both', help='Output format for MCAP conversion (LiDAR and GPS)')
	parser.add_argument('--image-format', choices=['images', 'csv', 'json', 'all'], default='images', help='Output format for image extraction')
	
	# Options pour types de données spécifiques
	parser.add_argument('--lidar-only', action='store_true', help='Convert only LiDAR data (skip GPS and Images)')
	parser.add_argument('--gps-only', action='store_true', help='Convert only GPS data (skip LiDAR and Images)')
	parser.add_argument('--images-only', action='store_true', help='Extract only Images (skip LiDAR and GPS)')
	
	# Options avancées
	parser.add_argument('--gps-topics', nargs='*', help='Custom GPS topics to search for')
	parser.add_argument('--image-topics', nargs='*', help='Custom image topics to search for')
	parser.add_argument('--skip-lidar', action='store_true', help='Skip LiDAR conversion')
	parser.add_argument('--skip-gps', action='store_true', help='Skip GPS conversion')
	parser.add_argument('--skip-images', action='store_true', help='Skip image extraction')
	
	args = parser.parse_args()
	

	if args.hostname and args.copy:
		parser.error("Cannot use both --hostname and --copy options together")
	
	if not args.skip_import and not args.hostname and not args.copy:
		parser.error("Either --hostname, --copy, or --skip_import option is required")
	
	if args.hostname and '@' not in args.hostname:
		parser.error("hostname(-u/--hostname) has an invalid syntax. Usage: user@guest-ip")
	
	if args.copy and not os.path.exists(args.copy):
		parser.error(f"Source directory '{args.copy}' does not exist")
	
	# Vérifier les conflits entre les options exclusives
	exclusive_options = [args.lidar_only, args.gps_only, args.images_only]
	if sum(exclusive_options) > 1:
		parser.error("Cannot use multiple exclusive options (--lidar-only, --gps-only, --images-only) together")
	
	if args.hostname:
		args.user = args.hostname.split('@')[0]
	
	return args

def get_remote_records(hostname, password):
	"""Get list of record directories on remote host."""
	logger.info(f"Retrieving record list from {hostname}...")
	try:
		cmd = f"sshpass -p {password} ssh {hostname} 'find  /home/$(whoami)/data/ -type d -mindepth 1 -maxdepth 1'"
		result = subprocess.run(cmd, shell=True, check=True, text=True, capture_output=True)
		records = result.stdout.strip().split('\n')
		
		if not records or (len(records) == 1 and not records[0]):
			logger.info("No record directories found on remote host.")
			return []
			
		logger.info(f"Found {len(records)} record directories.")
		return records
	except subprocess.CalledProcessError as e:
		logger.error(f"Error retrieving record list: {e}")
		logger.error(f"Command output: {e.stderr}")
		sys.exit(1)

def get_local_records(source_path):
	"""Get list of record directories from local source path."""
	logger.info(f"Looking for record directories in {source_path}...")
	try:
		entries = os.listdir(source_path)
		records = [os.path.join(source_path, entry) for entry in entries if os.path.isdir(os.path.join(source_path, entry))]
		
		if not records:
			logger.info("No record directories found in source path.")
			return []
			
		logger.info(f"Found {len(records)} record directories.")
		return records
	except Exception as e:
		logger.error(f"Error finding record directories: {e}")
		sys.exit(1)

def import_single_record(hostname, password, record, output_dir):
	"""Import a single record directory from remote host."""
	record_name = os.path.basename(record)
	target_dir = os.path.join(output_dir, record_name)
	
	if os.path.exists(target_dir):
		logger.info(f"Directory {target_dir} already exists, skipping import...")
		return target_dir
	
	logger.info(f"Importing {record_name} to {output_dir}...")
	os.makedirs(target_dir, exist_ok=True)
	
	try:
		cmd = f"sshpass -p {password} scp -r {hostname}:{record} {output_dir}/"
		logger.info(f"Running: {cmd}")
		subprocess.run(cmd, shell=True, check=True)
		
		logger.info(f"Successfully imported {record_name}")
		return target_dir
	except subprocess.CalledProcessError as e:
		logger.error(f"Error importing {record_name}: {e}")
		logger.error(f"Command output: {e.stderr}")
		return None

def copy_single_record(source_record, output_dir):
	"""Copy a single record directory from local source to output directory."""
	record_name = os.path.basename(source_record)
	target_dir = os.path.join(output_dir, record_name)
	
	if os.path.exists(target_dir) and record_name != "logs":
		logger.info(f"Directory {target_dir} already exists, skipping copy...")
		return target_dir
	
	logger.info(f"Copying {record_name} to {output_dir}...")
	
	try:
		shutil.copytree(source_record, target_dir, dirs_exist_ok=True)
		logger.info(f"Successfully copied {record_name}")
		return target_dir
	except Exception as e:
		logger.error(f"Error copying {record_name}: {e}")
		return None

def create_out_options_file(record_dir):
	"""Create out_options file for bag conversion."""
	out_options_content = """output_bags:
- uri: ./out/  # required
  storage_id: ""  # will use the default storage plugin, if unspecified
  max_bagfile_size: 0
  max_bagfile_duration: 0
  storage_preset_profile: ""
  storage_config_uri: ""
  all_topics: true
  topics: []
  topic_types: []
  all_services: true
  services: []
  all_actions: true
  actions: []
  rmw_serialization_format: ""
  regex: ""
  exclude_regex: ""
  exclude_topics: []
  exclude_topic_types: []
  exclude_services: []
  exclude_actions: []
  compression_mode: ""
  compression_format: ""
  compression_queue_size: 1
  compression_threads: 0
  include_hidden_topics: false
  include_unpublished_topics: false

"""
	out_options_path = os.path.join(record_dir, "out_options")
	with open(out_options_path, 'w') as f:
		f.write(out_options_content)
	return out_options_path

def convert_single_bag(record_dir):
	"""Convert rosbag files using ros2 bag convert command for a single record."""
	record_name = os.path.basename(record_dir)
	logger.info(f"Converting bags in {record_name}...")

	out_dir = os.path.join(record_dir, "out")

	if os.path.exists(os.path.join(out_dir, "metadata.yaml")):
		logger.info(f"Converted bag already exist in {record_dir}, skipping...")
		return True
	
	bag_dir = os.path.join(record_dir, "bag")
	if not os.path.exists(bag_dir):
		logger.info(f"Bag directory not found in {record_dir}, skipping...")
		return False	

	if os.path.exists(os.path.join(record_dir, "bag", "metadata.yaml")) == False:
		logger.warning(f"{record_dir} is missing a metadata.yaml")
		if repair_bag(os.path.join(record_dir, "bag")) == False:
			return False
	
	# Create out_options file
	out_options_path = create_out_options_file(record_dir)
	
	# Run conversion command
	try:
		cmd = f"ros2 bag convert -i {bag_dir} -o {out_options_path}"
		logger.info(f"Running: {cmd}")
		subprocess.run(cmd, shell=True, check=True, cwd=record_dir)
		logger.info(f"Successfully converted bags in {record_name}")
		return True
	except subprocess.CalledProcessError as e:
		logger.error(f"Error converting bags in {record_name}: {e}")
		return False

def repair_bag(bag_folder: str) -> bool:
	"""
		Unzip every compressed bag in bag_folder
		Return True if success
	"""
	print(f"Attempting to repair {bag_folder}")
	compressed_bags = glob.glob(os.path.join(bag_folder, "*.zstd"))
	try:
		for bag in compressed_bags:
			cmd = f"unzstd  {bag}"
			logger.info(f"Running: {cmd}")
			subprocess.run(cmd, shell=True, check=True, cwd=bag_folder)
		if len(compressed_bags) > 0:
			logger.info(f"Successfully uncompressed bag")
		cmd = "ros2 bag reindex ."
		logger.info(f"Running: {cmd}")
		subprocess.run(cmd, shell=True, check=True, cwd=bag_folder)
		logger.info(f"Bag {bag_folder} was repaired")
		return True
	except subprocess.CalledProcessError as e:
		logger.error(f"Error repairing bag in {bag_folder}: {e}")
	return False

def convert_mcap_data(record_dir, mcap_format="both", image_format="images", 
					 lidar_only=False, gps_only=False, images_only=False,
					 skip_lidar=False, skip_gps=False, skip_images=False,
					 custom_gps_topics=None, custom_image_topics=None):
	"""
	Convert MCAP data in the record directory using the API converters.
	
	Args:
		record_dir (str): Path to the record directory
		mcap_format (str): Output format for LiDAR and GPS ('json', 'csv', 'both')
		image_format (str): Output format for images ('images', 'csv', 'json', 'all')
		lidar_only (bool): Convert only LiDAR data
		gps_only (bool): Convert only GPS data
		images_only (bool): Convert only image data
		skip_lidar (bool): Skip LiDAR conversion
		skip_gps (bool): Skip GPS conversion
		skip_images (bool): Skip image extraction
		custom_gps_topics (list): Custom GPS topics to search for
		custom_image_topics (list): Custom image topics to search for
	
	Returns:
		dict: Conversion results with success/failure info
	"""
	if not MCAP_CONVERTERS_AVAILABLE:
		logger.warning("MCAP converters not available, skipping MCAP conversion")
		return {
			'lidar_success': False, 
			'gps_success': False, 
			'images_success': False,
			'error': 'Converters not available'
		}
	
	record_name = os.path.basename(record_dir)
	logger.info(f"Converting MCAP data in {record_name}...")
	
	results = {
		'lidar_success': False,
		'gps_success': False,
		'images_success': False,
		'lidar_stats': None,
		'gps_stats': None,
		'images_stats': None,
		'errors': []
	}
	
	# Vérifier si le dossier "out" existe (contient les MCAP) 
	out_dir = os.path.join(record_dir, "out")
	if not os.path.exists(out_dir):
		error_msg = f"No 'out' directory found in {record_dir}, skipping MCAP conversion"
		logger.warning(error_msg)
		results['errors'].append(error_msg)
		return results
	
	# Déterminer quoi convertir basé sur les options
	should_convert_lidar = not skip_lidar and not gps_only and not images_only
	should_convert_gps = not skip_gps and not lidar_only and not images_only
	should_convert_images = not skip_images and not lidar_only and not gps_only
	
	# Conversion LiDAR
	if should_convert_lidar and convert_lidar_batch:
		try:
			logger.info(f"Converting LiDAR data for {record_name}...")
			lidar_stats = convert_lidar_batch(
				root_directory=record_dir,
				output_format=mcap_format,
				verbose=False
			)
			
			results['lidar_success'] = lidar_stats['conversions_reussies'] > 0
			results['lidar_stats'] = lidar_stats
			
			if results['lidar_success']:
				logger.info(f"✓ LiDAR conversion successful: {lidar_stats['conversions_reussies']}/{lidar_stats['total_fichiers']} files")
			else:
				logger.warning(f"⚠ LiDAR conversion: no files converted")
				
		except Exception as e:
			if LidarConverterError and isinstance(e, LidarConverterError):
				error_msg = f"LiDAR conversion error for {record_name}: {e}"
			else:
				error_msg = f"Unexpected LiDAR conversion error for {record_name}: {e}"
			logger.error(error_msg)
			results['errors'].append(error_msg)
	
	# Conversion GPS
	if should_convert_gps and convert_gps_batch:
		try:
			logger.info(f"Converting GPS data for {record_name}...")
			gps_stats = convert_gps_batch(
				root_directory=record_dir,
				output_format=mcap_format,
				custom_topics=custom_gps_topics,
				verbose=False
			)
			
			results['gps_success'] = gps_stats['extractions_reussies'] > 0
			results['gps_stats'] = gps_stats
			
			if results['gps_success']:
				logger.info(f"✓ GPS conversion successful: {gps_stats['extractions_reussies']}/{gps_stats['total_fichiers']} files")
			else:
				logger.warning(f"⚠ GPS conversion: no files converted")
				
		except Exception as e:
			if GpsConverterError and isinstance(e, GpsConverterError):
				error_msg = f"GPS conversion error for {record_name}: {e}"
			else:
				error_msg = f"Unexpected GPS conversion error for {record_name}: {e}"
			logger.error(error_msg)
			results['errors'].append(error_msg)
	
	# Conversion Images
	if should_convert_images and convert_images_batch:
		try:
			logger.info(f"Converting image data for {record_name}...")
			images_stats = convert_images_batch(
				root_directory=record_dir,
				output_format=image_format,
				custom_topics=custom_image_topics,
				verbose=False
			)
			
			results['images_success'] = images_stats['extractions_reussies'] > 0
			results['images_stats'] = images_stats
			
			if results['images_success']:
				logger.info(f"✓ Images extraction successful: {images_stats['extractions_reussies']}/{images_stats['total_fichiers']} files, {images_stats['images_extraites']} images")
			else:
				logger.warning(f"⚠ Images extraction: no files converted")
				
		except Exception as e:
			if ImageConverterError and isinstance(e, ImageConverterError):
				error_msg = f"Images conversion error for {record_name}: {e}"
			else:
				error_msg = f"Unexpected images conversion error for {record_name}: {e}"
			logger.error(error_msg)
			results['errors'].append(error_msg)
	
	return results

def clean_local_bag_directory(record_dir):
	"""Delete the bag/ directory locally after successful processing."""
	bag_path = os.path.join(record_dir, "bag")
	if not os.path.exists(bag_path):
		return True
	try:
		shutil.rmtree(bag_path)
		return True
	except Exception as e:
		logger.error(f"Error cleaning local bag directory {bag_path}: {e}")
		return False

def process_single_record(record, output_dir, hostname=None, password=None, is_copy=False, clean=False, 
						 skip_mcap=False, mcap_format="both", image_format="images",
						 lidar_only=False, gps_only=False, images_only=False,
						 skip_lidar=False, skip_gps=False, skip_images=False,
						 custom_gps_topics=None, custom_image_topics=None):
	"""Process a single record: import/copy -> repair -> convert -> MCAP conversion -> clean (if enabled)."""
	record_name = os.path.basename(record)
	logger.info(f"Processing record: {record_name}")
	
	# Step 1: Import or copy
	if is_copy:
		target_dir = copy_single_record(record, output_dir)
	elif hostname:
		target_dir = import_single_record(hostname, password, record, output_dir)
	else:
		# Skip import case - record is already local
		target_dir = record
	
	if not target_dir:
		logger.error(f"Failed to import/copy {record_name}, skipping...")
		return None
	
	if record_name == "logs":
		return {'record_name': record_name, 'success': True, 'mcap_results': None}
	
	# Step 2: Convert ROS bags (includes repair if needed)
	conversion_success = convert_single_bag(target_dir)
	
	if not conversion_success:
		logger.error(f"Failed to convert ROS bags for {record_name}")
		return {'record_name': record_name, 'success': False, 'mcap_results': None}
	
	# Step 3: Convert MCAP data if enabled
	mcap_results = {'lidar_success': False, 'gps_success': False, 'images_success': False}
	if not skip_mcap and conversion_success:
		mcap_results = convert_mcap_data(
			target_dir, 
			mcap_format=mcap_format,
			image_format=image_format,
			lidar_only=lidar_only,
			gps_only=gps_only,
			images_only=images_only,
			skip_lidar=skip_lidar,
			skip_gps=skip_gps,
			skip_images=skip_images,
			custom_gps_topics=custom_gps_topics,
			custom_image_topics=custom_image_topics
		)
		
		# Log MCAP conversion summary
		conversions = []
		if mcap_results['lidar_success']:
			conversions.append("LiDAR")
		if mcap_results['gps_success']:
			conversions.append("GPS")
		if mcap_results['images_success']:
			conversions.append("Images")
			
		if conversions:
			logger.info(f"MCAP conversion completed for {record_name}: {', '.join(conversions)}")
		else:
			logger.warning(f"MCAP conversion had no successful outputs for {record_name}")
	
	# Step 4: Clean local bag directory if enabled and successful
	if clean and conversion_success:
		clean_local_bag_directory(target_dir)
	
	logger.info(f"Successfully processed record: {record_name}")
	return {'record_name': record_name, 'success': True, 'mcap_results': mcap_results}

def print_mcap_summary(all_mcap_results):
	"""Print a summary of all MCAP conversions."""
	if not all_mcap_results:
		return
	
	logger.info("\n" + "="*60)
	logger.info("MCAP CONVERSION SUMMARY")
	logger.info("="*60)
	
	# Filtrer les résultats valides
	valid_results = [r for r in all_mcap_results if r and r.get('mcap_results')]
	
	if not valid_results:
		logger.info("No MCAP conversions were performed.")
		return
	
	total_records = len(valid_results)
	lidar_successes = sum(1 for r in valid_results if r['mcap_results'].get('lidar_success', False))
	gps_successes = sum(1 for r in valid_results if r['mcap_results'].get('gps_success', False))
	images_successes = sum(1 for r in valid_results if r['mcap_results'].get('images_success', False))
	
	# Statistiques détaillées
	total_lidar_files = sum(r['mcap_results'].get('lidar_stats', {}).get('total_fichiers', 0) for r in valid_results if r['mcap_results'].get('lidar_stats'))
	total_gps_files = sum(r['mcap_results'].get('gps_stats', {}).get('total_fichiers', 0) for r in valid_results if r['mcap_results'].get('gps_stats'))
	total_image_files = sum(r['mcap_results'].get('images_stats', {}).get('total_fichiers', 0) for r in valid_results if r['mcap_results'].get('images_stats'))
	
	successful_lidar_files = sum(r['mcap_results'].get('lidar_stats', {}).get('conversions_reussies', 0) for r in valid_results if r['mcap_results'].get('lidar_stats'))
	successful_gps_files = sum(r['mcap_results'].get('gps_stats', {}).get('extractions_reussies', 0) for r in valid_results if r['mcap_results'].get('gps_stats'))
	successful_image_files = sum(r['mcap_results'].get('images_stats', {}).get('extractions_reussies', 0) for r in valid_results if r['mcap_results'].get('images_stats'))
	
	total_images_extracted = sum(r['mcap_results'].get('images_stats', {}).get('images_extraites', 0) for r in valid_results if r['mcap_results'].get('images_stats'))
	
	logger.info(f"Records processed: {total_records}")
	logger.info(f"LiDAR conversions successful: {lidar_successes}/{total_records} records")
	logger.info(f"GPS conversions successful: {gps_successes}/{total_records} records")
	logger.info(f"Image extractions successful: {images_successes}/{total_records} records")
	
	if total_lidar_files > 0:
		logger.info(f"LiDAR files processed: {successful_lidar_files}/{total_lidar_files}")
	if total_gps_files > 0:
		logger.info(f"GPS files processed: {successful_gps_files}/{total_gps_files}")
	if total_image_files > 0:
		logger.info(f"Image files processed: {successful_image_files}/{total_image_files}")
		logger.info(f"Total images extracted: {total_images_extracted}")
	
	# Show errors if any
	all_errors = []
	for result in valid_results:
		if result['mcap_results']:
			all_errors.extend(result['mcap_results'].get('errors', []))
	
	if all_errors:
		logger.warning(f"Total conversion errors: {len(all_errors)}")
		for error in all_errors[:5]:  # Show first 5 errors
			logger.warning(f"  - {error}")
		if len(all_errors) > 5:
			logger.warning(f"  ... and {len(all_errors) - 5} more errors")

def main():
	if os.environ.get("CYCLOSAFE_WORKSPACE", None) == None:
		print("[WARNING]: cyclosafe environment does not appear to be source. This is needed for the script to run correctly. To discard this warning and run the script without any cyclosafe environment, just export a CYCLOSAFE_WORPSACE environment variable using : \'export CYCLOSAFE_WORKSPACE=whatever\'")

	args = parse_arguments()
	
	# Vérifier la disponibilité des convertisseurs MCAP si nécessaire
	if not args.skip_mcap and not MCAP_CONVERTERS_AVAILABLE:
		logger.warning("MCAP converters not available. Install dependencies or check converter files.")
		logger.warning("Use --skip-mcap to proceed without MCAP conversion")
		response = input("Continue without MCAP conversion? (y/n): ")
		if response.lower() not in ['y', 'yes']:
			sys.exit(1)
		args.skip_mcap = True
	
	# Determine output directory
	if args.output:
		output_dir = os.path.abspath(args.output)
	else:
		output_dir = os.path.expanduser(f"~/data/import/")
	
	logger.info(f"Output directory: {output_dir}")
	os.makedirs(output_dir, exist_ok=True)
	
	# Get password if needed for remote operations
	password = None
	if args.hostname:
		password = getpass.getpass(prompt="Enter SSH password: ")
	
	# Track processing results
	all_results = []
	
	# Determine record directories to process
	if args.skip_import:
		# Use specified local path
		local_path = os.path.abspath(args.skip_import)
		if not os.path.exists(local_path):
			logger.error(f"Error: Specified path {local_path} does not exist")
			sys.exit(1)
		
		# Directory containing record directories specified
		records = glob.glob(os.path.join(local_path, "*-*"))
		
		if not records:
			logger.info(f"No record directories found in {local_path}")
			sys.exit(1)
		
		logger.info(f"Found {len(records)} record directories in {local_path}")
		
		# Process each record individually
		total_records = len(records)
		successful_records = 0
		for i, record in enumerate(records, 1):
			logger.info(f"[{i}/{total_records}] Processing record from local path...")
			result = process_single_record(
				record, output_dir, 
				clean=args.clean,
				skip_mcap=args.skip_mcap,
				mcap_format=args.mcap_format,
				image_format=args.image_format,
				lidar_only=args.lidar_only,
				gps_only=args.gps_only,
				images_only=args.images_only,
				skip_lidar=args.skip_lidar,
				skip_gps=args.skip_gps,
				skip_images=args.skip_images,
				custom_gps_topics=args.gps_topics,
				custom_image_topics=args.image_topics
			)
			if result and result['success']:
				successful_records += 1
			all_results.append(result)
		
	elif args.copy:
		# Import from local source path (e.g., mounted SD card)
		source_path = os.path.abspath(args.copy)
		local_records = get_local_records(source_path)
		
		if not local_records:
			logger.info("No records to copy. Exiting.")
			sys.exit(0)
		
		# Process each record individually
		total_records = len(local_records)
		successful_records = 0
		for i, record in enumerate(local_records, 1):
			logger.info(f"[{i}/{total_records}] Processing record from local copy...")
			result = process_single_record(
				record, output_dir, 
				is_copy=True, 
				clean=args.clean,
				skip_mcap=args.skip_mcap,
				mcap_format=args.mcap_format,
				image_format=args.image_format,
				lidar_only=args.lidar_only,
				gps_only=args.gps_only,
				images_only=args.images_only,
				skip_lidar=args.skip_lidar,
				skip_gps=args.skip_gps,
				skip_images=args.skip_images,
				custom_gps_topics=args.gps_topics,
				custom_image_topics=args.image_topics
			)
			if result and result['success']:
				successful_records += 1
			all_results.append(result)
		
	else:
		# Import from remote host
		hostname = args.hostname
		remote_records = get_remote_records(hostname, password)
		
		if not remote_records:
			logger.info("No records to import. Exiting.")
			sys.exit(0)
		
		# Process each record individually
		total_records = len(remote_records)
		successful_records = 0
		for i, record in enumerate(remote_records, 1):
			logger.info(f"[{i}/{total_records}] Processing record from remote host...")
			result = process_single_record(
				record, output_dir, 
				hostname=hostname, 
				password=password, 
				clean=args.clean,
				skip_mcap=args.skip_mcap,
				mcap_format=args.mcap_format,
				image_format=args.image_format,
				lidar_only=args.lidar_only,
				gps_only=args.gps_only,
				images_only=args.images_only,
				skip_lidar=args.skip_lidar,
				skip_gps=args.skip_gps,
				skip_images=args.skip_images,
				custom_gps_topics=args.gps_topics,
				custom_image_topics=args.image_topics
			)
			if result and result['success']:
				successful_records += 1
			all_results.append(result)
	
	# Print final summary
	logger.info(f"Processing completed! Successfully processed {successful_records} out of {total_records} records.")
	
	# Print MCAP conversion summary if conversions were performed
	if not args.skip_mcap:
		print_mcap_summary(all_results)

if __name__ == "__main__":
	main()