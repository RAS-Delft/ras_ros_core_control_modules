#!/usr/bin/env python

"""
Package with various operations related to transformations, often between local/global and cartesian/geographical coordinate systems
Also has some tools that help control, such as finding the smallest error angle between two angles.
"""


import numpy as np
from scipy import interpolate
import math
import os

def printTest():
	"""
	A test to check if the library is properly imported.
	"""
	print("ras_tf_lib1.py says hello!")
	print('directory of this file: is', os.path.dirname(os.path.realpath(__file__)))


def signed_shortest_angle_radians(angle1:float, angle2:float):
    """
    Calculate the signed shortest distance between two angles, in radians.

    Args:
    angle1 (int or float): The first angle, in radians.
    angle2 (int or float): The second angle, in radians.

    Returns:
    int or float: The signed shortest distance between the two angles, in radians. Sign dictates direction of rotation.
    """
    distance = (angle1 - angle2) % (2 * math.pi)
    if distance > math.pi:
        distance -= 2 * math.pi
    elif distance < -math.pi:
        distance += 2 * math.pi
    return distance

def d_northeast_to_d_latlong(dN:float,dE:float,lat_linearization:float):
	"""
	Transforms a small step in northeast tangent to earth surface in the difference in latitude longitude.
	Note: 
	- This method approximates earth locally flat and may misbehave around earths prime meredian or poles. 
	- Over long distances this method starts losing accuracy, but it works great and simple for small displacements. 
	- Approximates earth as a sphere

	Args:
		dN, dE float: movement north and east respectively in meters
		lat_linearization (float): latitude on which the linearization takes place. Usually taking the latitude of any near point suffices. 

	Returns:
		dlat,dlong float change in latitude and longitude respectively

	"""
	earth_radius = 6371.0e3  # Earth radius in meters
	r_lat = earth_radius * np.cos(np.radians(lat_linearization))
	dlat = np.degrees(dN/earth_radius)
	dlong = np.degrees(dE/r_lat)
	return dlat, dlong

def d_latlong_to_d_northeast(dlat:float,dlong:float,lat_linearization:float):
	"""
	Transforms a small step in latitude longitude to northeast motion tangent to earth surface.
	Note: 
	- This method approximates earth locally flat and may misbehave near earths prime meredian or poles. 
	- Over long distances this method starts losing accuracy, but it works great and simple for small displacements. 
	- Approximates earth as a sphere

	Args:
		dlat, dlong float: change in latitude and longitude respectively
		lat_linearization (float): latitude on which the linearization takes place. Usually taking the latitude of any near point suffices. 

	Returns:
		dN, dE float: movement north and east respectively in meters

	"""
	earth_radius = 6371.0e3  # Earth radius in meters
	r_lat = earth_radius * np.cos(np.radians(lat_linearization))
	dN = np.radians(dlat)*earth_radius
	dE = np.radians(dlong)*r_lat
	return dN, dE
	
def R_surf(yaw:float):
	"""
	Yields 2d rotation matrix about z axis of angle yaw
	"""
	return  np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])

def B_spline(waypoints:list,k_:int=4,s_:int=3):
	"""
	Smoothes a path along a set of waypoints using a B spline approach

	Made according to guide at:
	https://www.youtube.com/watch?v=ueUgHvUT2Z0

	params: 
	- waypoints: list of waypoints with first axis points, second axis has lat,long at index 0,1 respectively
	- k (optional): smoothing order
	- s (optional): smoothness factor
	"""

	x = []
	y = []
	
	for point in waypoints:
		x.append(point[0])
		y.append(point[1])
		
	# order k= 3 and smoothness s=0
	tck, *rest = interpolate.splprep([x,y],k=k_,s=s_)
	# tck is a tuple containing ths spline's knots, coefficients and degree defining our smoothed path

	# make the smooth spline into a set of waypoints
	u = np.linspace(0,1,num=400) # vector stating how far we are into our path
	smooth = interpolate.splev(u,tck)
	return smooth

def euler_from_quaternion(x:float, y:float, z:float, w:float):
	"""
	Convert a quaternion into euler angles (roll, pitch, yaw)
	roll is rotation around x in radians (counterclockwise)
	pitch is rotation around y in radians (counterclockwise)
	yaw is rotation around z in radians (counterclockwise)
	"""
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + y * y)
	roll_x = math.atan2(t0, t1)
	
	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	pitch_y = math.asin(t2)
	
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (y * y + z * z)
	yaw_z = math.atan2(t3, t4)
	
	return roll_x, pitch_y, yaw_z # in radians

def geo_transform_surface(pose_init,transform):
	"""
	Transforms pose_init across surface of earth (linearized over a short distance)
	Initial pose is transformed by first doing translation and secondly rotation. 
	The transformation operation is expressed in initial coordinate system
	
	Args:
	pose_init (float[3]): The initial pose of the object [latitude (deg), longitude (deg), yaw(radians)]
	transform (float[3]): The transformation from object to new coordinate system [dx (meters), dy (meter), d_yaw (radians)]

	Returns:
	pose_out (float[3])
	"""

	yaw_formation = pose_init[2] - transform[2]
	R_formation = R_surf(yaw_formation)
	dn_de = -np.matmul(R_formation,transform[0:2])

	dlat_dlong = delta_northeast_to_delta_longlat(dn_de,pose_init[0])

	lat_long_new = np.array(pose_init[0:2]) + dlat_dlong
	return np.concatenate((lat_long_new, np.array([yaw_formation])), axis=0)

def delta_northeast_to_delta_longlat(dn_de,lat):
	"""
	Calculates displacement in degrees (longitude & latitude) if the object would move a certain amount of meters 
	tangent to earth surface north and east.

	Args:
	dn_de (float[2]): vector with 2 elements pertaining movement north and east respectively
	lat (float): latitude on which the linearization takes place. Any near point. 

	Returns:def errorangleunwrap(a,aref):
	
	while a<0:
		a +=2*math.pi
	while aref<0:
		aref+=2*math.pi
	while a>2*math.pi:
		a-= 2*math.pi
	while aref>2*math.pi:
		aref-=2*math.pi
		
	if aref>a:
		if aref-a<math.pi:
			e=aref-a
		else:
			e = aref-a -2*math.pi
	else:
		if a - aref <math.pi:
			e = aref - a
		else:
			e = aref - a +2*math.pi

	return e
	[dlat,dlong] (float[2]) change in latitude and longitude respectively
	"""
	earth_radius = 6371.0e3  # Earth radius in meters
	r_lat = earth_radius * np.cos(np.radians(lat))
	dlat = np.degrees(dn_de[0]/earth_radius)
	dlong = np.degrees(dn_de[1]/r_lat)
	return np.array([dlat,dlong])

def errorangleunwrap(a,aref):
	
	while a<0:
		a +=2*math.pi
	while aref<0:
		aref+=2*math.pi
	while a>2*math.pi:
		a-= 2*math.pi
	while aref>2*math.pi:
		aref-=2*math.pi
		
	if aref>a:
		if aref-a<math.pi:
			e=aref-a
		else:
			e = aref-a -2*math.pi
	else:
		if a - aref <math.pi:
			e = aref - a
		else:
			e = aref - a +2*math.pi

	return e

def chaikins_corner_cutting(points:np.ndarray, refinements:int=1):
    """
    Apply Chaikin's corner cutting algorithm to a set of points.
    Generally used to smooth out a polygon. (e.g. vessel path)
    """
    for _ in range(refinements):
        L = len(points)
        new_points = np.zeros((2*L, 2))
        new_points[0] = points[0]
        new_points[-1] = points[-1]
        for i in range(0, L):
            # Get the points from the list, looping around if the index is out of bounds
            p0 = points[(i-1) % L]
            p1 = points[(i) % L]
            q0 = p0 + 1./4 * (p1 - p0)
            q1 = p0 + 3./4 * (p1 - p0)
            new_points[2*i-1] = q0
            new_points[2*i] = q1
        points = new_points
    return points

def chaikins_corner_cutting2(coords, refinements:int=5):
    """
    Check which of these two implementations is faster
    
    Apply Chaikin's corner-cutting algorithm to a set of coordinates.
    params:
    - coords: list of coordinates
    - refinements (optional): number of refinements to perform
    
    Author of this formulation: Liran Funaro
    https://stackoverflow.com/questions/47068504/where-to-find-python-implementation-of-chaikins-corner-cutting-algorithm
	"""
    coords = np.array(coords)
    for _ in range(refinements):
        L = coords.repeat(2, axis=0)
        R = np.empty_like(L)
        R[0] = L[0]
        R[2::2] = L[1:-1:2]
        R[1:-1:2] = L[2::2]
        R[-1] = L[-1]
        coords = L * 0.75 + R * 0.25

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