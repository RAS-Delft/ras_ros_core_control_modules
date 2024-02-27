import rclpy
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
import time
import numpy as np

import ras_ros_core_control_modules.tools.geometry_tools as ras_geometry_tools
import ras_ros_core_control_modules.tools.display_tools as ras_display_tools

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
		super().__init__('pos_vel_differentiation_node')
	
		self.declare_parameters(
            namespace='',
            parameters=[
                ('period_broadcast_status', 5.0),
				('minimum_differentiation_timestep',0.1),
				]
        )

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
		self.differentiate_semaphore = 0
		self.posVel = None 
		self.yawVel = None
		
		# Filters
		self.filt_u = StreamingMovingAverage(8)
		self.filt_v = StreamingMovingAverage(8)
		self.filt_r = StreamingMovingAverage(8)

		# Start publishers and subscribers
		self.subscription_pos = self.create_subscription(NavSatFix,'telemetry/gnss/fix',self.input_pos_callback,10)
		self.subscription_yaw = self.create_subscription(Float32,'telemetry/heading',self.input_yaw_callback,10)
		self.publisher_vel = self.create_publisher(Float32MultiArray, 'state/velocity', 10)
		self.publisher_vel_unfiltered = self.create_publisher(Float32MultiArray, 'state/velocity_unfiltered', 10)

		# Diagnostics
		self.timer_statistics_last = self.get_clock().now().nanoseconds/1e9
		self.timer_status = self.create_timer(self.get_parameter('period_broadcast_status').get_parameter_value().double_value, self.print_statistics)
		self.tracker_num_pos_updates = 0
		self.tracker_num_yaw_updates = 0
		self.tracker_num_diffs = 0
		self.tracker_num_mainloop_callback = 0
		
	def input_pos_callback(self, msg):
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
		"""
		Differentiates the position and heading data to get velocity data.
		Applies a moving average filter to the velocies to reduce noise, and publishes the filtered and unfiltered velocities.
		Some conditionals are in place to avoid differentiating with faulty data, such as with startup of the code or very small timesteps.
		"""
		if self.updatedPos and self.updatedYaw and not self.differentiate_semaphore: # Check if both position and heading have had an update and another callback did not trigger this function yet.
			self.differentiate_semaphore = 1 # Block the use of this function by other callbacks until this function is finished
			if self.previousTimeYaw and self.previousTimePos: # check if these values have initial values, else, do not differentiate but set them

				delta_t_yaw = self.newTimeYaw -self.previousTimeYaw
				delta_t_pos = self.newTimePos -self.previousTimePos

				if delta_t_yaw>self.get_parameter('minimum_differentiation_timestep').get_parameter_value().double_value and delta_t_pos>self.get_parameter('minimum_differentiation_timestep').get_parameter_value().double_value:
					# add to tracker
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
					
					# Filtering x y yaw with moving average filter  ----------------
					vel_filtered = [0,0,0]
					vel_filtered[0] = self.filt_u.process(vel[0])
					vel_filtered[1] = self.filt_v.process(vel[1])
					vel_filtered[2] = self.filt_r.process(vel[2])
					
					# Publish
					msg = Float32MultiArray(data=vel_filtered)
					self.publisher_vel.publish(msg) # Publish filtered velocity
					msg = Float32MultiArray(data=vel)
					self.publisher_vel_unfiltered.publish(msg) # Publish unfiltered velocity

					# Finalizing - setting 'previousvalues' for the next iteration
					self.previousTimeYaw = self.newTimeYaw
					self.previousYaw = self.newYaw
					self.previousTimePos = self.newTimePos
					self.previousPos = self.newPos
			else:
				# If either of the previous values is None, set them to the current values (this is the case in the first iteration of the code)
				self.previousTimeYaw = self.newTimeYaw
				self.previousYaw = self.newYaw
				self.previousTimePos = self.newTimePos
				self.previousPos = self.newPos

			self.differentiate_semaphore = 0 # release semaphore


			'''
			# Semaphohre blocking this function while it is in progress
			# I do not know why I saw results indicating that multiple callbacks could do this. Implementing this semaphore fixed the issue, although more in depth research could be nice on this. 
			# Seek to remove this blocking feature in the future if possible, or seek out to reproduce the issue. 
			self.differentiate_semaphore = 1 # Semaphore for this function

			self.updatedPos = 0 # Boolean if updated since last publication
			self.updatedYaw = 0

			# Check if there are previous values existing on yaw and heading (In the first iteration(s) this is not the case, in which case these values equal 'None')
			# Only differentiate after multiple measurements of heading and position have been recorded
			if self.previousTimeYaw and self.previousTimePos:
				print('checkA')
				# Calculate timestep for the measurements
				delta_t_yaw = self.newTimeYaw -self.previousTimeYaw
				delta_t_pos = self.newTimePos -self.previousTimePos

		
				# Only run if a minimum timestep has passed (this is to avoid corrupted data due to occasional extremely small timesteps)
				if delta_t_yaw>self.get_parameter('minimum_differentiation_timestep').get_parameter_value().double_value and delta_t_pos>self.get_parameter('minimum_differentiation_timestep').get_parameter_value().double_value:
					print('checkB')
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
			self.differentiate_semaphore = 0 # release semaphore
			'''
		
	
	def print_statistics(self):
		""" On a single line, print the rates of all major callbacks in this script. """

		# Calculate passed time
		now = self.get_clock().now().nanoseconds/1e9
		passed_time = now - self.timer_statistics_last

		# Determine system frequencies rounded to two decimals
		freq_pos = round(self.tracker_num_pos_updates / passed_time, 2)
		freq_yaw = round(self.tracker_num_yaw_updates / passed_time, 2)
		freq_diff = round(self.tracker_num_diffs / passed_time, 2)
		
		# Format information to string
		printstring = ras_display_tools.terminal_fleet_module_string(self.get_namespace()[1:], ['freq_pos', freq_pos, 'Hz'], ['freq_yaw', freq_yaw, 'Hz'], ['freq_diff', freq_diff, 'Hz'])
												
		# Print
		self.get_logger().info(printstring)

		# Reset trackers  
		self.tracker_num_pos_updates = 0
		self.tracker_num_yaw_updates = 0
		self.tracker_num_diffs = 0
		self.timer_statistics_last = now


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