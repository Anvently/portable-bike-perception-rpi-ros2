#!/usr/bin/env python3
"""
Decryption and decompression script for encrypted ROS bag files
Compatible with hybrid RSA+AES encryption and zstd compression
FIXED VERSION - Properly removes metadata from the decrypted file
"""

import os
import logging
import argparse
import json
from pathlib import Path
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
import zstandard as zstd


class ROSBagDecryptor:
	"""
	Class for decrypting and decompressing encrypted ROS bag files
	"""
	
	def __init__(self, private_key_path):
		self.private_key = self._load_private_key(private_key_path)
		logging.info("Private key successfully loaded")
	
	def _load_private_key(self, private_key_path):
		"""Load the RSA private key from a PEM file"""
		try:
			with open(private_key_path, 'rb') as key_file:
				private_key = serialization.load_pem_private_key(
					key_file.read(),
					password=None,
				)
			return private_key
		except Exception as e:
			logging.error(f"Error loading private key: {e}")
			raise
	
	def decrypt_file(self, encrypted_file_path, output_dir=None):
		"""
		Decrypt an encrypted file and return ONLY the original data
		without encryption metadata
		"""
		encrypted_file_path = Path(encrypted_file_path)
				
		try:
			with open(encrypted_file_path, 'rb') as infile:
				# Read metadata
				metadata_length = int.from_bytes(infile.read(4), 'big')
				metadata_json = infile.read(metadata_length)
				metadata = json.loads(metadata_json.decode('utf-8'))
				
				logging.debug(f"Metadata: {metadata}")
				
				# Read encrypted AES key
				encrypted_aes_key = infile.read(metadata['key_length'])
				
				# Read IV
				iv = infile.read(metadata['iv_length'])
				
				# Read encrypted data (ONLY the original encrypted data)
				encrypted_data = infile.read()
			
			# Decrypt AES key
			aes_key = self.private_key.decrypt(
				encrypted_aes_key,
				padding.OAEP(
					mgf=padding.MGF1(algorithm=hashes.SHA256()),
					algorithm=hashes.SHA256(),
					label=None
				)
			)
			
			# Decrypt data
			cipher = Cipher(algorithms.AES(aes_key), modes.CBC(iv))
			decryptor = cipher.decryptor()
			decrypted_padded = decryptor.update(encrypted_data) + decryptor.finalize()
			
			# Remove PKCS7 padding
			pad_length = decrypted_padded[-1]
			decrypted_data = decrypted_padded[:-pad_length]
			
			# Decompress if needed
			if metadata.get('compressed', True):
				compression_method = metadata.get('compression_method', 'zstd')
				
				if compression_method == 'zstd':
					dctx = zstd.ZstdDecompressor()
					original_data = dctx.decompress(decrypted_data)
				else:
					logging.warning(f"Unknown compression method: {compression_method}")
					original_data = decrypted_data
			else:
				original_data = decrypted_data
			
			# Define output directory
			if output_dir:
				output_dir = Path(output_dir)
				output_dir.mkdir(parents=True, exist_ok=True)
			else:
				output_dir = encrypted_file_path.parent
			
			# Save decrypted file with ONLY original data
			output_file = output_dir / metadata['original_name']
			
			with open(output_file, 'wb') as outfile:
				outfile.write(original_data)
			
			logging.info(f"Decrypted file saved: {output_file}")
			return output_file
			
		except Exception as e:
			logging.error(f"Error decrypting {encrypted_file_path}: {e}")
			raise
	
	def decrypt_directory(self, encrypted_dir, output_dir=None):
		"""
		Decrypt all .enc files in a directory
		"""
		encrypted_dir = Path(encrypted_dir)
		
		if not encrypted_dir.exists():
			raise FileNotFoundError(f"Directory not found: {encrypted_dir}")
		
		# Find all .enc files
		encrypted_files = list(encrypted_dir.glob("*.enc"))
		
		if not encrypted_files:
			logging.warning(f"No .enc files found in {encrypted_dir}")
			return []
		
		decrypted_files = []
		
		for encrypted_file in encrypted_files:
			try:
				output_file = self.decrypt_file(encrypted_file, output_dir)
				decrypted_files.append(output_file)
			except Exception as e:
				logging.error(f"Unable to decrypt {encrypted_file}: {e}")
		
		logging.info(f"Decryption finished: {len(decrypted_files)}/{len(encrypted_files)} files")
		return decrypted_files
	
	def verify_file_integrity(self, encrypted_file_path):
		"""
		Verify the integrity of an encrypted file (metadata)
		"""
		try:
			with open(encrypted_file_path, 'rb') as infile:
				# Read only metadata
				metadata_length = int.from_bytes(infile.read(4), 'big')
				metadata_json = infile.read(metadata_length)
				metadata = json.loads(metadata_json.decode('utf-8'))
				
				required_fields = ['original_name', 'encryption_method', 'key_length', 'iv_length']
				
				for field in required_fields:
					if field not in metadata:
						return False, f"Missing field: {field}"
				
				if metadata['encryption_method'] != 'RSA-OAEP + AES-256-CBC':
					return False, f"Unsupported encryption method: {metadata['encryption_method']}"
				
				return True, "Valid file"
				
		except Exception as e:
			return False, f"Read error: {e}"
	
	def get_file_info(self, encrypted_file_path):
		"""
		Display information of an encrypted file without decrypting it
		"""
		try:
			with open(encrypted_file_path, 'rb') as infile:
				metadata_length = int.from_bytes(infile.read(4), 'big')
				metadata_json = infile.read(metadata_length)
				metadata = json.loads(metadata_json.decode('utf-8'))
				
				print(f"Encrypted file info: {encrypted_file_path}")
				print(f"  Original name: {metadata['original_name']}")
				print(f"  Encryption method: {metadata['encryption_method']}")
				print(f"  Compression method: {metadata.get('compression_method', 'None')}")
				print(f"  Timestamp: {metadata.get('timestamp', 'Unknown')}")
				print(f"  Encrypted key size: {metadata['key_length']} bytes")
				print(f"  IV size: {metadata['iv_length']} bytes")
				
				# Compute encrypted data size
				infile.seek(0, 2)  # Go to end
				total_size = infile.tell()
				data_size = total_size - 4 - metadata_length - metadata['key_length'] - metadata['iv_length']
				print(f"  Encrypted data size: {data_size} bytes")
				
		except Exception as e:
			print(f"Error reading file info: {e}")


def main():
	parser = argparse.ArgumentParser(description='ROS bag file decryptor')
	parser.add_argument('--private-key', required=True, help='Path to private key')
	parser.add_argument('--file', help='Encrypted file to decrypt')
	parser.add_argument('--directory', help='Directory containing encrypted files')
	parser.add_argument('--output-dir', help='Output directory for decrypted files')
	parser.add_argument('--verify', action='store_true', help='Verify file integrity only')
	parser.add_argument('--info', action='store_true', help='Show info of encrypted files')
	parser.add_argument('--verbose', '-v', action='store_true', help='Verbose mode')
	
	args = parser.parse_args()
	
	# Logging configuration
	level = logging.INFO if args.verbose else logging.WARNING
	logging.basicConfig(
		level=level,
		format='%(asctime)s - %(levelname)s - %(message)s'
	)
	
	if not args.file and not args.directory:
		print("Error: You must specify either --file or --directory")
		return
	
	if not os.path.exists(args.private_key):
		print(f"Error: Private key not found: {args.private_key}")
		return
	
	try:
		decryptor = ROSBagDecryptor(args.private_key)
		
		if args.file:
			# Decrypt a single file
			if not os.path.exists(args.file):
				print(f"Error: File not found: {args.file}")
				return
			
			if args.verify:
				is_valid, message = decryptor.verify_file_integrity(args.file)
				print(f"Verification of {args.file}: {message}")
				return
			
			if args.info:
				decryptor.get_file_info(args.file)
				return
			
			output_file = decryptor.decrypt_file(args.file, args.output_dir)
			print(f"Decrypted file: {output_file}")
		
		elif args.directory:
			# Decrypt a directory
			if not os.path.exists(args.directory):
				print(f"Error: Directory not found: {args.directory}")
				return
			
			if args.verify:
				encrypted_files = list(Path(args.directory).glob("*.enc"))
				if not encrypted_files:
					print(f"No .enc files found in {args.directory}")
					return
				
				for encrypted_file in encrypted_files:
					is_valid, message = decryptor.verify_file_integrity(encrypted_file)
					print(f"Verification of {encrypted_file.name}: {message}")
				return
			
			if args.info:
				encrypted_files = list(Path(args.directory).glob("*.enc"))
				if not encrypted_files:
					print(f"No .enc files found in {args.directory}")
					return
				
				for encrypted_file in encrypted_files:
					decryptor.get_file_info(encrypted_file)
					print("-" * 50)
				return
			
			decrypted_files = decryptor.decrypt_directory(args.directory, args.output_dir)
			print(f"Decryption finished: {len(decrypted_files)} files processed")
			
			for file_path in decrypted_files:
				print(f"  - {file_path}")
	
	except Exception as e:
		logging.error(f"Error: {e}")
		print(f"Error during decryption: {e}")


if __name__ == "__main__":
	main()
