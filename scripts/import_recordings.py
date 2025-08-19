#!/usr/bin/env python3
# Created on Tue Aug 05 2025 by Nicolas Pirard
# Updated on Tue Aug 13 2025 by Eric Ta
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
from decryptor import ROSBagDecryptor
from bag_converter import RosBagExporter
import time

os.environ['NUMEXPR_MAX_THREADS'] = '8'

FORMAT = "%(message)s"
logging.basicConfig(
	level="INFO", format=FORMAT, datefmt="[%X]", handlers=[RichHandler()]
)  # set level=20 or logging.INFO to turn off debug
logger = logging.getLogger("rich")

def parse_arguments():
	"""Parse command line arguments."""
	parser = argparse.ArgumentParser(description='Import, decompress and merge rosbag files from a remote host to single bag file. Convert MCAP data to CSV/JSON/Images. Attempt to repair corrupted bags.')
	parser.add_argument('-u', '--hostname', help='Hostname for SSH connection (user@host-ip)')
	parser.add_argument('-c', '--copy', help='Path to a local source directory (e.g., mounted SD card) to copy from')
	parser.add_argument('-s', '--skip_import', help='Skip import step and use specified local path for conversion')
	parser.add_argument('-o', '--output', help='Output directory for imported bags')
	parser.add_argument('-x', '--clean', action='store_true', help='Delete bag/ directories locally after successful import and conversion')

	parser.add_argument('--skip-conversion', action='store_true', help='Skip MCAP conversion (LiDAR, GPS, and Images)')
	
	args = parser.parse_args()

	if args.hostname and args.copy:
		parser.error("Cannot use both --hostname and --copy options together")
	
	if not args.skip_import and not args.hostname and not args.copy:
		parser.error("Either --hostname, --copy, or --skip_import option is required")
	
	if args.hostname and '@' not in args.hostname:
		parser.error("hostname(-u/--hostname) has an invalid syntax. Usage: user@guest-ip")
	
	if args.copy and not os.path.exists(args.copy):
		parser.error(f"Source directory '{args.copy}' does not exist")

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
	"""Convert rosbag files using ros2 bag convert command for a single record.
	Take recording directory and create out directory to store conversion. Does not override existing out/ dir"""
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
	try:
		cmd = "ros2 bag reindex ."
		logger.info(f"Running: {cmd}")
		subprocess.run(cmd, shell=True, check=True, cwd=bag_folder)
		logger.info(f"Bag {bag_folder} was repaired")
		return True
	except subprocess.CalledProcessError as e:
		logger.error(f"Error repairing bag in {bag_folder}: {e}")
	return False

def decrypt_bag_folder(decryptor, bag_folder):
	"""Decrypt every .enc folder in bag_folder. Return True for success"""
	encrypted_files = glob.glob(os.path.join(bag_folder, "*.mcap.enc"))
	success = True
	for file in encrypted_files:
		try:
			result, error = decryptor.verify_file_integrity(file)
			if result == False:
				raise Exception(f"Verifying file integrity: {error}")
			decryptor.decrypt_file(file)
		except Exception as e:
			logger.error(f"Decrypter: file integrity check failed for {file}: {e}")
			success = False
	return success

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

def process_single_record(record, output_dir, hostname=None, password=None, is_copy=False, clean=False, skip_conversion=False) -> bool:
	"""Process a single record: import/copy -> decrypt + decompress -> repair -> convert -> MCAP to CSV conversion -> clean (if enabled)."""
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
		return False
	
	if record_name == "logs":
		return True
	
	# Step 2: Decrypt and decompress bags
	private_key_path = os.getenv("PRIVATE_KEY_PATH")
	if private_key_path == None:
		logger.error(f"Private key path not specified. Cannot decrypt files in {record_name}")
		return False
	decryptor = ROSBagDecryptor(private_key_path)
	if decrypt_bag_folder(decryptor, os.path.join(target_dir, 'bag')) == False:
		return False

	# Step 3: Convert ROS bags (includes repair if needed)
	conversion_success = convert_single_bag(target_dir)
	
	if not conversion_success:
		logger.error(f"Failed to convert ROS bags for {record_name}")
		return False
	
	# Step 4: Convert MCAP data if enabled
	if skip_conversion == False:
		if not os.path.exists(os.path.join(target_dir, "export")):
			with RosBagExporter(os.path.join(target_dir, "out"), os.path.join(target_dir, "export")) as exporter:
				exporter.process_bag()

		else:
			logger.info(f"Exported data for {record_name} already exist. Skipping...")

		
	# Step 5: Clean local bag directory if enabled and successful
	if clean and conversion_success:
		clean_local_bag_directory(target_dir)
	
	logger.info(f"Successfully processed record: {record_name}")
	return True

def main():
	if os.environ.get("CYCLOSAFE_WORKSPACE") is None:
		logger.warning("[WARNING]: cyclosafe environment does not appear to be source. "
			"This is needed for the script to run correctly. "
			"To discard this warning and run the script without any cyclosafe environment, just run: "
			"export CYCLOSAFE_WORKSPACE=whatever")

	if os.environ.get("PRIVATE_KEY_PATH") is None:
		logger.warning("[WARNING]: variable PRIVATE_KEY_PATH is not defined in env. "
			"This is needed to decrypt imported data. The script will not run until the end. "
			"Starting in 3s... Ctrl-C to terminate.")
		time.sleep(3)

	args = parse_arguments()


	# Output directory
	output_dir = os.path.abspath(args.output) if args.output else os.path.expanduser("~/data/import/")
	logger.info(f"Output directory: {output_dir}")
	os.makedirs(output_dir, exist_ok=True)

	# Password if remote
	password = getpass.getpass("Enter SSH password: ") if args.hostname else None

	if args.skip_import:
		local_path = os.path.abspath(args.skip_import)
		if not os.path.exists(local_path):
			logger.error(f"Error: Specified path {local_path} does not exist")
			sys.exit(1)
		records = glob.glob(os.path.join(local_path, "*-*"))
		if not records:
			logger.info(f"No record directories found in {local_path}")
			sys.exit(1)
		record_source = "local path"
		record_kwargs = dict()
	elif args.copy:
		source_path = os.path.abspath(args.copy)
		records = get_local_records(source_path)
		if not records:
			logger.info("No records to copy. Exiting.")
			sys.exit(0)
		record_source = "local copy"
		record_kwargs = dict(is_copy=True)
	else:
		hostname = args.hostname
		records = get_remote_records(hostname, password)
		if not records:
			logger.info("No records to import. Exiting.")
			sys.exit(0)
		record_source = "remote host"
		record_kwargs = dict(hostname=hostname, password=password)

	total_records = len(records)
	successful_records = 0

	for i, record in enumerate(records, 1):
		logger.info(f"[{i}/{total_records}] Processing record from {record_source}...")
		if process_single_record(
			record,
			output_dir,
			clean=args.clean,
			skip_conversion=args.skip_conversion,
			**record_kwargs
		):
			successful_records += 1

	# Résumé final
	logger.info(f"Processing completed! Successfully processed {successful_records} out of {total_records} records.")


if __name__ == "__main__":
	main()