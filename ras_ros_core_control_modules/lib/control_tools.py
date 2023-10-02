#!/usr/bin/env python3

import time
import numpy as np
from std_msgs.msg import Float32MultiArray

def generate_rosmsg_PID_status(pid):
	""" generates a float32MultiArray ros message with the current PID status """
	msg = Float32MultiArray()
	# [reference, error, output (last_y), proportional output (last_yp), integral output (last_yi), derivative output (last_yd), timestep (dt), integral buildup (error_integral), state]
	msg.data = [pid.ref, pid.last_error, pid.last_y, pid.last_yp, pid.last_yi, pid.last_yd, pid.last_dt, pid.error_integral, pid.state]
	return msg

class PIDController():
	"""
	A real-time PID controller featuring integral buildup limiting, output bounding
	"""
	def __init__(self,Kp:float,Ki:float,Kd:float,**kwargs):
		self.kp = Kp
		self.ki = Ki
		self.kd = Kd

		# Set kwargs
		self.integral_limits = kwargs['integral_limits'] if 'integral_limits' in kwargs else []
		self.output_limits = kwargs['output_limits'] if 'output_limits' in kwargs else []
		self.ref = kwargs['ref_init'] if 'ref_init' in kwargs else 0
		self.state = kwargs['state_init'] if 'state_init' in kwargs else 0
		self.timeout = kwargs['timeout'] if 'timeout' in kwargs else 0

		self.error_integral = 0
		self.last_error = 0
		self.tlast = time.time()
		self.available = 1 # Semaphore to prevent multiple calls to compute() before the previous call has finished

		# memory of last values for diagnostics

		self.last_dt = 0
		self.last_yp = 0
		self.last_yi = 0
		self.last_yd = 0
		self.last_y = 0
		
	def setRef(self,ref:float):
		self.ref = ref
		
	def setState(self,state:float):
		self.state = state
		
	def reset(self):
		self.tlast = time.time()
		self.error_integral = 0
		self.last_error = 0
		self.available = 1
	
	def calc_error(self):
		return self.ref - self.state

	def compute(self):
		"""
		Compute PID controller output
	
		"""
		if self.available:
			now = time.time()
			if self.timeout:
				if now-self.tlast>self.timeout:
					self.reset()
				
			if self.tlast:
				self.available = 0
				
				dt = now-self.tlast

				error = self.calc_error()
				self.error_integral += error * dt
				
				# Limit integrator buildup
				if self.integral_limits:
					if self.error_integral < self.integral_limits[0]:
						self.error_integral = self.integral_limits[0]
					elif self.error_integral > self.integral_limits[1]:
						self.error_integral = self.integral_limits[1]
				
				if dt>0:
					error_derivative = (error - self.last_error) / dt
				else: 
					error_derivative =0
					print('[WARNING] PIDController somehow tried to differentiate over a timestep of '+str(dt)+'seconds')
				
				yp = self.kp * error
				yi = self.ki * self.error_integral
				yd = self.kd * error_derivative
				out = yp + yi + yd

				# Save last values for diagnostics
				self.last_dt = dt
				self.last_yp = yp
				self.last_yi = yi
				self.last_yd = yd
				self.last_y = out

				# Save last error for derivative calculation next iteration
				self.last_error = error
				
				# Limit output
				if self.output_limits:
					if out < self.output_limits[0]:
						out = self.output_limits[0]
					elif out > self.output_limits[1]:
						out = self.output_limits[1]
				
				self.tlast = now
				self.available = 1
				return out
			else:
				self.tlast = time.time()