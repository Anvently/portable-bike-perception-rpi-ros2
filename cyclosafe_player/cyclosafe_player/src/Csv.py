

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

