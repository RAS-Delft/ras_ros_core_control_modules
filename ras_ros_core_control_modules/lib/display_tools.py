#!/usr/bin/env python3
""" Display related functionalies"""

class rascolors:
	"""
	Color lookup table for terminal output
	"""

	# Status colors
	# ----------------
	HEADER = '\033[95m'
	OKBLUE = '\033[94m'
	OKCYAN = '\033[96m'
	OKGREEN = '\033[92m'
	WARNING = '\033[93m'
	FAIL = '\033[91m'
	NORMAL = '\033[0m'

	# Vessel colors
	# ----------------
	# Dark blue Tito Neri
	RAS_TN_DB = '\033[38;5;21m'
	# Light blue Tito Neri
	RAS_TN_LB = '\033[38;5;39m'
	# Green Tito Neri
	RAS_TN_GR = '\033[38;5;22m'
	# Yellow Tito Neri
	RAS_TN_YE = '\033[38;5;226m'
	# Red Tito Neri
	RAS_TN_RE = '\033[38;5;196m'
	# Orange Tito Neri
	RAS_TN_OR = '\033[38;5;208m'
	# Violet Tito Neri
	RAS_TN_PU = '\033[38;5;141m'

	# Undefined ship as grey
	VESSEL_COLOR_DEFAULT = '\033[38;5;242m'