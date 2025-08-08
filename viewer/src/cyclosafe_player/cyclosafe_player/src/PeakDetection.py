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
# https://cecill.info/licences/Licence_CeCILL-B_V1-en.html

from typing import List, Dict, Tuple, Any
import numpy as np

class Peak:
	def __init__(self, time: float, range: float):
		self.start_time: float = time
		self.values: List[float] = [range]
		self.duration: float = None
		self.min: float = None
		self.max: float = None
		self.mean: float = None
		self.sample_count: int = 0

	def add_sample(self, time: float, range: float, keep_range: bool = True):
		if keep_range:
			self.values.append(range)
		self.duration = time - self.start_time

	def evaluate(self, end_time):
		np_array = np.array(self.values)
		self.min = np_array.min()
		self.max = np_array.max()
		self.mean = np_array.mean()
		self.std_dev = np_array.std()
		self.sample_count = len(self.values)
		self.duration = end_time - self.start_time

	def __str__(self):
		return f"duration={self.duration}\nmean={self.mean}\nstd dev={self.std_dev}\nmin={self.min}\nmax={self.max}\nsamples={self.sample_count}"
		

def compute_baseline(sonar_data) -> float:
	if len(sonar_data) >= 10:
		baseline = sum([r for _, r in sonar_data[:10]]) / 10
	else:
		baseline = sum([r for _, r in sonar_data]) / len(sonar_data)
	return baseline

def detect_peaks(sonar_data, threshold_drop=3.0, min_duration_sec=0.3, recovery_tolerance_sec=0.3, feedback_handler=None):
	"""
	Détecte les pics dans les données d'un sonar (chutes soudaines de distance)
	
	Args:
		sonar_data: liste de tuples (temps_rel, range_value)
		threshold_drop: seuil en dessous duquel une valeur est considérée comme un pic (par rapport à la valeur normale)
		min_duration_sec: durée minimale d'un pic pour être considéré valide (en secondes)
		recovery_tolerance_sec: durée maximale d'un rétablissement qui ne met pas fin au pic (en secondes)
	
	Returns:
		liste de dictionnaires contenant les infos sur chaque pic détecté
	"""
	if not sonar_data:
		return []

	peaks = []
	current_peak: Peak = None
	in_recovery = False
	recovery_start_time = 0
	last_feedback = 0
	total_duration = sonar_data[-1][0] if len(sonar_data) > 0 else 0 
	
	for i, (time, range_val) in enumerate(sonar_data):
		# Début potentiel d'un pic
		if range_val < threshold_drop and current_peak is None:
			current_peak = Peak(time, range_val)
			# current_peak = Peak(sonar_data[i - 1][0] if i != 0 else sonar_data[i][0], range_val)

		# Pendant un pic potentiel
		elif current_peak is not None:
			if range_val < threshold_drop:
				# Toujours dans le pic, ajouter la valeur
				current_peak.add_sample(time, range_val)
				
				# Si on était en période de rétablissement temporaire, on ne l'est plus
				in_recovery = False
				
			elif range_val >= threshold_drop:  # range_val >= min_peak_drop
				if not in_recovery:
					# Début d'une période de rétablissement potentielle
					in_recovery = True
					recovery_start_time = time
					
				# Vérifier si la durée de rétablissement dépasse la tolérance
				if in_recovery and (time - recovery_start_time) > recovery_tolerance_sec:
					if current_peak.duration >= min_duration_sec:
						current_peak.evaluate(recovery_start_time)
						if current_peak.min <= threshold_drop:
							peaks.append(current_peak)
					
					# Réinitialiser pour chercher le prochain pic
					current_peak = None
					in_recovery = False
				else:
					# On est en rétablissement temporaire, on ignore la valeur
					current_peak.add_sample(time, range_val, False)

		if feedback_handler:
			if time - last_feedback > 10:
				feedback_handler(f"{time}/{total_duration}")
				last_feedback = time

	
	# Gérer le cas où on est encore dans un pic à la fin des données
	if current_peak is not None:
		current_peak.evaluate(sonar_data[-1][0])
		if current_peak.min <= threshold_drop:
			peaks.append(current_peak)
	
	return peaks


def analyze_multiple_sonars(sonars_data):
	"""
	Analyse les données de plusieurs sonars et détecte les pics pour chacun
	
	Args:
		sonars_data: dictionnaire {nom_sonar: données_sonar}
	
	Returns:
		dictionnaire avec les résultats d'analyse pour chaque sonar
	"""
	results = {}
	
	for sonar_name, sonar_data in sonars_data.items():
		peaks = detect_peaks(sonar_data)
		
		results[sonar_name] = {
			'total_peaks': len(peaks),
			'peaks': peaks
		}
		
	return results

