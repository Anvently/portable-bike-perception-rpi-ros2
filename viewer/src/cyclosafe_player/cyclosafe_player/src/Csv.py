# Created on Tue Aug 05 2025
# Updated on Tue Aug 05 2025
#
#  This file is part of Cyclosafe
# Copyright (c) 2025 Nicolas Pirard @Anvently
#
# This software is governed by the CeCILL license under French law and
# abiding by the rules of distribution of free software. You can use,
# modify and/or redistribute the software under the terms of the CeCILL
# license as circulated by CEA, CNRS and INRIA at:
# https://cecill.info/licences/Licence_CeCILL_V2.1-en.html


def parse_csv_format(format_text):
	"""Analyse le texte de format et retourne un dictionnaire de mapping"""
	format_mapping = {}
	current_idx = 0
	for part in format_text.strip().split(','):
		part = part.strip()
		if '$' in part:
			idx_part, attr_part = part.split('$', 1)
			idx = int(idx_part.strip())
			if idx < current_idx:
				raise Exception(f"Inconsistant numerotation: index {idx} of {part} following column {current_idx}")
			current_idx = idx
			
			if '=' in attr_part:
				attr, label = attr_part.split('=', 1)
				format_mapping[current_idx] = {'attr': attr.strip(), 'label': label.strip()}
			else:
				format_mapping[current_idx] = {'attr': attr_part.strip(), 'label': attr_part.strip()}
		elif part.isdigit():
			idx = int(part.strip())
			if idx < current_idx:
				raise Exception(f"Inconsistant numerotation: index {idx} of {part} following column {current_idx}")
			current_idx = idx
		else:
			raise Exception(f"Invalid token {part}")
			
	return format_mapping

