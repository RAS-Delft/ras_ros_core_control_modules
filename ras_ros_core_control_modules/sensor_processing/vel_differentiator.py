import rclpy
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
import time
import numpy as np
import argparse

import ras_ros_core_control_modules.tools.geometry_tools as ras_geometry_tools
import ras_ros_core_control_modules.tools.display_tools as ras_display_tools

parser = argparse.ArgumentParser(description='ROS2 control module for differentiation of position and heading to velocity')
parser.add_argument("objectID", type=str,help="set vessel identifier")
parser.add_argument('-r') # ROS2 arguments
args, unknown = parser.parse_known_args()

# Set constants
OBJECT_ID = args.objectID
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
	
class DifferentiationNode(Node):
	def __init__(self):
		super().__init__('differentiation_node')
	
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
		self.subscription_pos = self.create_subscription(NavSatFix,'telemetry/gnss/fix',self.input_pos_callback,10)
		self.subscription_yaw = self.create_subscription(Float32,'telemetry/heading',self.input_yaw_callback,10)
		self.publisher_vel = self.create_publisher(Float32MultiArray, 'state/velocity', 10)
		self.publisher_vel_unfiltered = self.create_publisher(Float32MultiArray, 'state/velocity_unfiltered', 10)
		
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
					delta_yaw = ras_geometry_tools.signed_shortest_angle_radians(self.newYaw,self.previousYaw)
					yawVel = delta_yaw / delta_t_yaw
					
					# Calculating linear velocities ----------------
					# Calculate the north-east displacement in meters
					#d_ne = spherical_to_ne(self.previousPos[0], self.previousPos[1], self.newPos[0], self.newPos[1])
					
					dlat = self.newPos[0] - self.previousPos[0]
					dlon = self.newPos[1] - self.previousPos[1]
					dN,dE = ras_geometry_tools.d_latlong_to_d_northeast(dlat,dlon,self.previousPos[0])
					
					# Divide by timestep to get velocities north and east direction
					vel_ne = np.array([ dN/delta_t_pos, dE/delta_t_pos])
					
					# Rotate linear velocities to get body fixed velocities
					R = ras_geometry_tools.R_surf(-self.newYaw)
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
		
	
	def print_statistics(self):
		""" On a single line, print the rates of all major callbacks in this script. """

		# Calculate passed time
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
def main(args=None):
	rclpy.init(args=args)

	node = DifferentiationNode()

	# Start the nodes processing thread
	rclpy.spin(node)

	# at termination of the code (generally with ctrl-c) Destroy the node explicitly
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()