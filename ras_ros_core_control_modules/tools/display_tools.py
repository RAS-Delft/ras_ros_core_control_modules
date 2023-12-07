#!/usr/bin/env python3
""" Display related functionalies"""


class vesselcolors_rgb:
	"""
	RGB Color code lookup table for vessel colors
	scale: 0-255	
	"""
	RAS_TN_DB = [0, 27, 230]
	RAS_TN_LB = [63, 155, 252]
	RAS_TN_GR = [31, 135, 12]
	RAS_TN_YE = [217, 211, 30]
	RAS_TN_RE = [209, 24, 10]
	RAS_TN_OR = [247, 132, 0]
	RAS_TN_PU = [183, 0, 255]
	DEFAULT = [130, 130, 130]

	def nameSelect(name:str):
		"""
		Check if name is a field of this class, and return the corresponding color, else default"""
		if hasattr(vesselcolors_rgb, name):
			return getattr(vesselcolors_rgb, name)
		else:
			return vesselcolors_rgb.DEFAULT

	def get_normalized(name:str):
		"""
		Returns a list with the RGB values of the color in a normalized range (0-1)
		"""
		rgbvalue = vesselcolors_rgb.nameSelect(name)
		#return [rgbvalue[0]/255.0, rgbvalue[1]/255.0, rgbvalue[2]/255.0]
		# round to three decimals
		return [round(rgbvalue[0]/255.0,3), round(rgbvalue[1]/255.0,3), round(rgbvalue[2]/255.0,3)]
	
	def get_normalized_string(name:str):
		"""
		Returns a string with the RGB values of the color in a normalized range (0-1)
		"""
		rgbvalue_normalized = vesselcolors_rgb.get_normalized(name)
		return str(rgbvalue_normalized[0])+" "+str(rgbvalue_normalized[1])+" "+str(rgbvalue_normalized[2])


def rgb_to_ansi(rgb:list):
	"""
	Converts an RGB list to an ANSI color code
	"""
	return '\x1b[38;2;'+str(rgb[0])+';'+str(rgb[1])+ ';'+str(rgb[2])+'m'

class statuscolors_ansi:
	"""
	ANSI Color code lookup table for terminal output
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

class vesselcolors_ansi:
	"""
	ANSI Color code lookup table for terminal output
	"""
	RAS_TN_DB = rgb_to_ansi(vesselcolors_rgb.RAS_TN_DB)
	RAS_TN_LB = rgb_to_ansi(vesselcolors_rgb.RAS_TN_LB)
	RAS_TN_GR = rgb_to_ansi(vesselcolors_rgb.RAS_TN_GR)
	RAS_TN_YE = rgb_to_ansi(vesselcolors_rgb.RAS_TN_YE)
	RAS_TN_RE = rgb_to_ansi(vesselcolors_rgb.RAS_TN_RE)
	RAS_TN_OR = rgb_to_ansi(vesselcolors_rgb.RAS_TN_OR)
	RAS_TN_PU = rgb_to_ansi(vesselcolors_rgb.RAS_TN_PU)
	VESSEL_COLOR_DEFAULT = rgb_to_ansi(vesselcolors_rgb.DEFAULT)

	def id_to_color(name:str):
		"""
		Check if name is a field of this class, and return the corresponding color, else default"""
		if hasattr(vesselcolors_ansi, name):
			return getattr(vesselcolors_ansi, name)
		else:
			return vesselcolors_ansi.VESSEL_COLOR_DEFAULT	

def terminal_fleet_module_string(objectID:str, *items):
	"""
	Formats a string for terminal output of a fleet module, with the format shown below (on a single line)
	object ID and values are colored according to the rascolors class. 

	<SHIP ID> 
	<value1 name> = <value1> <value1 unit> 
	<value2 name> = <value2> <value2 unit> 
	...

	param objectID: The name of the object (e.g. ship identifier)
	param items: Variable number of lists [value name:str, value:int/float, unit:str (optional)]
	return: The formatted string

	"""
	# Start with the object name (e.g. ship identifier)
	returnstring = vesselcolors_ansi.id_to_color(objectID) + objectID + statuscolors_ansi.NORMAL

	# Add the value name, value and unit for each pair
	for item in items:
		returnstring += " " + item[0] + "=" # Value name
		if item[1] > 0:	# Value
			returnstring += statuscolors_ansi.OKGREEN + str(round(item[1],2)) + statuscolors_ansi.NORMAL
		else:
			returnstring += statuscolors_ansi.FAIL + str(round(item[1],2)) + statuscolors_ansi.NORMAL
		if len(item) > 2:
			returnstring += "" + item[2] # Unit

	# Return the combined output
	return returnstring