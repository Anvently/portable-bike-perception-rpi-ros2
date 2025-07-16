#!/usr/bin/env python3

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

def parse_arguments():
	"""Parse command line arguments."""
	parser = argparse.ArgumentParser(description='Import, decompress and merge rosbag files from a remote host to single bag file. Attemmpt to repair corrupted bags.')
	parser.add_argument('-u', '--hostname', help='Hostname for SSH connection (user@host-ip)')
	parser.add_argument('-c', '--copy', help='Path to a local source directory (e.g., mounted SD card) to copy from')
	parser.add_argument('-s', '--skip_import', help='Skip import step and use specified local path for conversion')
	parser.add_argument('-o', '--output', help='Output directory for imported bags')
	parser.add_argument('-x', '--clean', action='store_true', help='Delete bag/ directories locally after successful import and conversion')
	
	args = parser.parse_args()
	
	# Validate arguments
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
		# Find directories that match the pattern (assuming *-* pattern as in the original code)
		records = glob.glob(os.path.join(source_path, "*-*"))
		
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
	
	if os.path.exists(target_dir):
		logger.info(f"Directory {target_dir} already exists, skipping copy...")
		return target_dir
	
	logger.info(f"Copying {record_name} to {output_dir}...")
	
	try:
		shutil.copytree(source_record, target_dir)
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
  # optional filter for msg time t [nsec since epoch]:  start_time_ns <= t <= end_time_ns
  # start_time_ns: 1744227144744197147
  # end_time_ns: 1744227145734665546
  all_topics: true
  topics: []
  topic_types: []
  all_services: true
  services: []
  all_actions: true
  actions: []
  rmw_serialization_format: ""  # defaults to using the format of the input topic
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

def process_single_record(record, output_dir, hostname=None, password=None, is_copy=False, clean=False):
	"""Process a single record: import/copy -> repair -> convert -> clean (if enabled)."""
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
	
	# Step 2: Convert (includes repair if needed)
	conversion_success = convert_single_bag(target_dir)
	
	if not conversion_success:
		logger.error(f"Failed to convert {record_name}")
		return False
	
	# Step 3: Clean local bag directory if enabled and successful
	if clean and conversion_success:
		clean_local_bag_directory(target_dir)
	
	logger.info(f"Successfully processed record: {record_name}")
	return True

def main():
	args = parse_arguments()
	
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
			success = process_single_record(record, output_dir, clean=args.clean)
			if success:
				successful_records += 1
		
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
			success = process_single_record(record, output_dir, is_copy=True, clean=args.clean)
			if success:
				successful_records += 1
		
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
			success = process_single_record(record, output_dir, hostname=hostname, password=password, clean=args.clean)
			if success:
				successful_records += 1
	
	logger.info(f"Processing completed! Successfully processed {successful_records} out of {total_records} records.")

if __name__ == "__main__":
	main()