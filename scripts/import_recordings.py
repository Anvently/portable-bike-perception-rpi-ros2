#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys
import glob
import getpass
import logging
from rich.logging import RichHandler

FORMAT = "%(message)s"
logging.basicConfig(
	level="NOTSET", format=FORMAT, datefmt="[%X]", handlers=[RichHandler()]
)  # set level=20 or logging.INFO to turn off debug
logger = logging.getLogger("rich")

def parse_arguments():
	"""Parse command line arguments."""
	parser = argparse.ArgumentParser(description='Import and convert rosbag files from a remote host.')
	parser.add_argument('-u', '--hostname', help='Hostname for SSH connection (user@host-ip)')
	parser.add_argument('-s', '--skip_import', help='Skip import step and use specified local path for conversion')
	parser.add_argument('-o', '--output', help='Output directory for imported bags')
	
	args = parser.parse_args()
	
	# Validate arguments
	if not args.skip_import and not args.hostname:
		parser.error("hostname argument is required unless --skip_import is specified")
	
	if not args.skip_import:
		if '@' in args.hostname:
			args.user = args.hostname.split('@')[0]
		else:
			parser.error("hostname(-u/--hostname) as an invalid syntaxe. Usage: user@guest-ip")
	
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

def import_records(hostname, password, records, output_dir):
	"""Import record directories from remote host."""
	os.makedirs(output_dir, exist_ok=True)
	
	total_records = len(records)
	for i, record in enumerate(records, 1):
		record_name = os.path.basename(record)
		target_dir = os.path.join(output_dir, record_name)
		
		if os.path.exists(target_dir):
			logger.info	(f"[{i}/{total_records}] Directory {target_dir} already exists, skipping...")
			continue
		
		logger.info	(f"[{i}/{total_records}] Importing {record_name} to {output_dir}...")
		os.makedirs(target_dir, exist_ok=True)
		
		try:
			cmd = f"sshpass -p {password} scp -r {hostname}:{record} {output_dir}/"
			logger.info	(f"Running: {cmd}")
			subprocess.run(cmd, shell=True, check=True)
			
			logger.info	(f"Successfully imported {record_name}")
		except subprocess.CalledProcessError as e:
			logger.error(f"Error importing {record_name}: {e}")
			logger.error(f"Command output: {e.stderr}")
			continue
	
	return output_dir

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

def convert_bags(record_dirs):
	"""Convert rosbag files using ros2 bag convert command."""
	total_records = len(record_dirs)
	
	for i, record_dir in enumerate(record_dirs, 1):
		record_name = os.path.basename(record_dir)
		logger.info(f"[{i}/{total_records}] Converting bags in {record_name}...")
		
		bag_dir = os.path.join(record_dir, "bag")
		if not os.path.exists(bag_dir):
			logger.info(f"Bag directory not found in {record_dir}, skipping...")
			continue
		
		out_dir = os.path.join(record_dir, "out")

		if os.path.exists(os.path.join(out_dir, "metadata.yaml")):
			logger.info(f"Converted bag already exist in {record_dir}, skipping...")
			continue

		if os.path.exists(os.path.join(record_dir, "bag", "mtadata.yaml")) == False:
			logger.warning(f"{record_dir} is missing a metadata.yaml")
			if repair_bag(os.path.join(record_dir, "bag")) == False:
				continue
		
		# Create out_options file
		out_options_path = os.path.join(record_dir, "out_options")
		out_options_path = create_out_options_file(record_dir)
		
		# Run conversion command
		try:
			cmd = f"ros2 bag convert -i {bag_dir} -o {out_options_path}"
			logger.info(f"Running: {cmd}")
			subprocess.run(cmd, shell=True, check=True, cwd=record_dir)
			logger.info(f"Successfully converted bags in {record_name}")
		except subprocess.CalledProcessError as e:
			logger.error(f"Error converting bags in {record_name}: {e}")
			continue

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

def main():
	args = parse_arguments()
	
	# Determine output directory
	if args.output:
		output_dir = os.path.abspath(args.output)
	else:
		output_dir = os.path.expanduser(f"~/data/import/")
	
	logger.info(f"Output directory: {output_dir}")
	
	# Determine record directories to process
	if args.skip_import:
		# Use specified local path
		local_path = os.path.abspath(args.skip_import)
		if not os.path.exists(local_path):
			logger.error(f"Error: Specified path {local_path} does not exist")
			sys.exit(1)
		
		# Directory containing record directories specified
		record_dirs = glob.glob(os.path.join(local_path, "*-*"))
		
		if not record_dirs:
			logger.info(f"No record directories found in {local_path}")
			sys.exit(1)
		
		logger.info(f"Found {len(record_dirs)} record directories in {local_path}")
	else:
		# Import from remote host
		if '@' in args.hostname:
			hostname = args.hostname
		else:
			hostname = f"{args.user}@{args.hostname}"
		
		password = getpass.getpass(prompt="Enter SSH password: ")
		remote_records = get_remote_records(hostname, password)
		
		if not remote_records:
			logger.info("No records to import. Exiting.")
			sys.exit(0)
		
		import_records(hostname, password, remote_records, output_dir)
		# Convert record_dirs from string to list of directories
		record_dirs = glob.glob(os.path.join(output_dir, "*-*"))
	
	# Convert bags
	convert_bags(record_dirs)
	logger.info("All operations completed successfully!")

if __name__ == "__main__":
	main()