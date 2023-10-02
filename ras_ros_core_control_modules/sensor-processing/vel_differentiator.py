import rospy
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import NavSatFix
import time
import math
import numpy as np
import argparse
import ras_tf_lib.ras_tf_lib1  as rtf
from ras_tf_lib.ras_tf_lib1 import rascolors as rascolors

parser = argparse.ArgumentParser(description='get input of ship ID')
parser.add_argument('vessel_id', type=str, nargs=1,help='Vessel Identifier of the specified ship')
args, unknown = parser.parse_known_args()

MINIMUM_DIFFERENTIATE_TIMESTEP = 0.1 # seconds
PERIOD_BROADCAST_STATUS = 5.0 #seconds

class StreamingMovingAverage:
	"""
	A discrete real-time moving average filter
	Saves last n=window_size values of a datastream, and returns the average as the output. 
	"""
	def __init__(self, window_size):
		self.window_size = window_size
		self.values = []
		self.sum = 0

	def process(self, value):
		"""
		Process a new input value, return the average of the last n=window_size values
		"""
		self.values.append(value)
		self.sum += value
		if len(self.values) > self.window_size:
			self.sum -= self.values.pop(0)
		return float(self.sum) / len(self.values)
	
class DifferentiationNode():
	def __init__(self,vessel_id_):


		self.vessel_id = vessel_id_
		self.node = rospy.init_node('differentiation_node', anonymous=True)
		
		# Look if vessel_id is a field of the rascolors class. Set self.color to the corresponding color or otherwise VESSEL_COLOR_DEFAULT
		self.color = getattr(rascolors, self.vessel_id, rascolors.VESSEL_COLOR_DEFAULT)

		# Set memory for differentiation purposes
		self.previousPos = None # previous values
		self.previousYaw = None
		self.newPos = None
		self.newYaw = None
		self.previousTime = None
		self.previousTimePos = None # Timestamps of receiving of previous value
		self.previousTimeYaw = None
		self.newTimePos = None
		self.newTimeYaw = None
		
		self.tStart = time.time()
		self.timestamp_broadcast_status = self.tStart
		
		self.updatedPos = 0 
		self.updatedYaw = 0
		self.busy = 0
		self.posVel = None 
		self.yawVel = None
		

		# Diagnostics
		self.tracker_num_pos_updates = 0
		self.tracker_num_yaw_updates = 0
		self.tracker_num_diffs = 0
		self.tracker_num_mainloop_callback = 0
		
		# Filters
		self.filt_u = StreamingMovingAverage(8)
		self.filt_v = StreamingMovingAverage(8)
		self.filt_r = StreamingMovingAverage(8)

		# Start publishers and subscribers
		self.subscription_pos = rospy.Subscriber('/'+self.vessel_id+'/state/geopos',NavSatFix,self.input_pos_callback)
		self.subscription_yaw = rospy.Subscriber('/'+self.vessel_id+'/state/yaw',Float32,self.input_yaw_callback)
		self.publisher_vel = rospy.Publisher('/'+self.vessel_id+'/state/velocity',Float32MultiArray, queue_size=1)
		self.publisher_vel_unfiltered = rospy.Publisher('/'+self.vessel_id+'/state/velocity_unfiltered',Float32MultiArray, queue_size=1)
		
	def input_pos_callback(self, msg):
		#print('position callback: '+"{:.5f}".format(msg.latitude)+' '+"{:.5f}".format(msg.longitude))
		self.tracker_num_pos_updates += 1
		self.newTimePos = time.time()
		self.newPos = [msg.latitude,msg.longitude]
		self.updatedPos = 1
		self.differentiate()
		
	def input_yaw_callback(self, msg):
		"""
		Runs when a ROS message has been published on the heading topic
		It 
		"""
		self.tracker_num_yaw_updates += 1
		if msg.data:
			self.newTimeYaw = time.time()
			self.newYaw = msg.data	
			self.updatedYaw = 1
			self.differentiate()
		else:
			print('[WARNING][input_yaw_callback] Faulty yaw data detected at ! msg.data =')
			print('----')
			print(msg.data)
			print('----')

	def differentiate(self):
		if self.updatedPos and self.updatedYaw and not self.busy: # Check if both position and heading have had an update and another callback did not trigger this function yet.
			
			self.updatedPos = 0 # Boolean if updated since last publication
			self.updatedYaw = 0
			
			# Blocking another callback to start this function while it is still running.
			# I do not know why I saw results indicating that multiple callbacks could do this. 
			# Seek to remove this blocking feature in the future if possible, or seek out the 
			self.busy = 1
			
			# Check if there are previous values existing on yaw and heading (In the first iteration(s) this is not the case, in which case these values equal 'None')
			# Only differentiate after multiple measurements of heading and position have been recorded
			if self.previousTimeYaw and self.previousTimePos:
				
				# Calculate timestep for the measurements
				delta_t_yaw = self.newTimeYaw -self.previousTimeYaw
				delta_t_pos = self.newTimePos -self.previousTimePos
				
				# Only run if a minimum timestep has passed (this is to avoid corrupted data due to occasional extremely small timesteps)
				if delta_t_yaw>MINIMUM_DIFFERENTIATE_TIMESTEP and delta_t_pos>MINIMUM_DIFFERENTIATE_TIMESTEP:
					self.tracker_num_diffs += 1
					# Calculating angular velocity ----------------
					delta_yaw = rtf.signed_shortest_angle_radians(self.newYaw,self.previousYaw)
					yawVel = delta_yaw / delta_t_yaw
					
					# Calculating linear velocities ----------------
					# Calculate the north-east displacement in meters
					#d_ne = spherical_to_ne(self.previousPos[0], self.previousPos[1], self.newPos[0], self.newPos[1])
					
					dlat = self.newPos[0] - self.previousPos[0]
					dlon = self.newPos[1] - self.previousPos[1]
					dN,dE = rtf.d_latlong_to_d_northeast(dlat,dlon,self.previousPos[0])
					
					# Divide by timestep to get velocities north and east direction
					vel_ne = np.array([ dN/delta_t_pos, dE/delta_t_pos])
					
					# Rotate linear velocities to get body fixed velocities
					R = rtf.R_surf(-self.newYaw)
					posVel = np.matmul(R,vel_ne)
					
					# Concatenate X Y and heading ----------------
					vel = posVel.tolist()+[yawVel]
					
					# Filtering with moving average filter  ----------------
					vel_filtered = [0,0,0]
					vel_filtered[0] = self.filt_u.process(vel[0])
					vel_filtered[1] = self.filt_v.process(vel[1])
					vel_filtered[2] = self.filt_r.process(vel[2])
					
					# Publish
					msg = Float32MultiArray(data=vel_filtered)
					self.publisher_vel.publish(msg)
					msg = Float32MultiArray(data=vel)
					self.publisher_vel_unfiltered.publish(msg)
					
			# Finalizing - setting 'previousvalues' for the next iteration
			self.previousTimeYaw = self.newTimeYaw
			self.previousYaw = self.newYaw
			self.previousTimePos = self.newTimePos
			self.previousPos = self.newPos
			self.busy = 0 # release this function availability 
		
	
	def run(self):
		rate = rospy.Rate(1000)  # Hz
		while not rospy.is_shutdown():
			now = time.time()
			self.tracker_num_mainloop_callback += 1

			if now - self.timestamp_broadcast_status > PERIOD_BROADCAST_STATUS:
				self.timestamp_broadcast_status = now

				# Determine system frequencies rounded to two decimals
				freq_pos = round(self.tracker_num_pos_updates / PERIOD_BROADCAST_STATUS, 2)
				freq_yaw = round(self.tracker_num_yaw_updates / PERIOD_BROADCAST_STATUS, 2)
				freq_diff = round(self.tracker_num_diffs / PERIOD_BROADCAST_STATUS, 2)

				# Make strings of numbers without color if above zero and and rascolors.FAIL otherwise
				freq_pos_str = rascolors.OKGREEN +str(freq_pos) + rascolors.NORMAL if freq_pos > 0 else rascolors.FAIL + str(freq_pos) + rascolors.NORMAL
				freq_yaw_str = rascolors.OKGREEN +str(freq_yaw) + rascolors.NORMAL if freq_yaw > 0 else rascolors.FAIL + str(freq_yaw) + rascolors.NORMAL
				freq_diff_str = rascolors.OKGREEN +str(freq_diff) + rascolors.NORMAL if freq_diff > 0 else rascolors.FAIL + str(freq_diff) + rascolors.NORMAL

				# Print system state
				print(' '+self.color + self.vessel_id + rascolors.NORMAL +' [velocity differentiator]['+str(round(time.time()-self.tStart,2)) + 's] freq_pos: ' + freq_pos_str + ' Hz, freq_yaw: ' + freq_yaw_str + ' Hz, freq_diff: ' + freq_diff_str + ' Hz')

				self.tracker_num_pos_updates = 0
				self.tracker_num_yaw_updates = 0
				self.tracker_num_diffs = 0


			rate.sleep()


if __name__ == '__main__':
	try:
		node = DifferentiationNode(args.vessel_id[0])
		node.run()
	except rospy.ROSInterruptException:
		pass
