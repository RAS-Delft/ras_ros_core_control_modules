'''
This control effort allocator subclasses control_effort_allocator_super.py and overrides the allocate_forces method to implement a simple control effort allocation protocol.
'''

import rclpy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Wrench


# import titoneri_parameters.py
import ras_ros_core_control_modules.tools.titoneri_parameters  as titoneri_parameters

## Import the superclass
from ras_ros_core_control_modules.control_effort_allocation.control_effort_allocator_super import ControlEffortAllocatorNode

def skew_symmetric_matrix(vector):
	"""Calculates the skew symmetric matrix of a vector
		Args:
			vector: 3x1 vector
		Returns:
			3x3 skew symmetric matrix
	"""
	return np.array([[0, -vector[2], vector[1]],
					 [vector[2], 0, -vector[0]],
					 [-vector[1], vector[0], 0]])

def calculate_thrust_configuration_vector(thrusterangle,thusterposition):
	"""Calculates the thrust configuration vector for a single thruster
		Args:
			thrusterangle: angle of the thruster (around z axis, radians)
			thrusterposition: position of the thruster (2x1 vector x,y, meters)
		Returns:
			thrust configuration vector (3x1)
	"""
	t = np.array([	np.cos(thrusterangle),
			   		np.sin(thrusterangle),
					thusterposition[0]*np.sin(thrusterangle) - thusterposition[1]*np.cos(thrusterangle)])
	
	# Vertical array
	t = t.reshape(3,1)

	return t

class ControlEffortAllocator_DP1(ControlEffortAllocatorNode):
	""" 
	ROS2 control module for allocation of control efforts to actuators.
	This particular version has fixed angles of the thrusters and uses simple matrix multiplication to figure out thrust per actuator and sends this
	"""
	def __init__(self):
		super().__init__('dp_control_effort_allocator')

		# Initialize the aft_thruster_type parameter that is 'black_model' by default
		# This is used to look up the actuator force input relation later on
		self.declare_parameter('thruster_type', 'aft_black')

		# Set thruster angles in radians
		self.alpha = np.array([0.0, np.pi/2, np.pi/2]) # SB, PS, Bow

		# Set thruster positions
		self.r_th = titoneri_parameters.thruster_locations() # SB, PS, Bow
		
		# Initialize the forces per thrusters vector as float32
		self.f_thrusters = np.zeros(3, dtype=np.float32)


		# set the actuator force input relation # SB, PS, Bow
		self.actuator_force_input_relation = [titoneri_parameters.TN_force_to_thrusterusage1(self.get_parameter('thruster_type').get_parameter_value().string_value),
											  titoneri_parameters.TN_force_to_thrusterusage1(self.get_parameter('thruster_type').get_parameter_value().string_value),
											  titoneri_parameters.TN_force_to_thrusterusage1('bowThruster')]

		# Set force to actuation input relation # SB, PS, Bow
		self.force_actuation_input_relation = [titoneri_parameters.TN_thrusterusage_to_force1(self.get_parameter('thruster_type').get_parameter_value().string_value),
											   titoneri_parameters.TN_thrusterusage_to_force1(self.get_parameter('thruster_type').get_parameter_value().string_value),
											   titoneri_parameters.TN_thrusterusage_to_force1('bowThruster')]

		# Set the actuator input limits
		self.thrust_limits = titoneri_parameters.actuator_input_limits()
		self.force_allocation_limits = titoneri_parameters.force_allocation_limits()

		# Set the thruster allocation matrix 
		t1 = calculate_thrust_configuration_vector(self.alpha[0],self.r_th[0])
		t2 = calculate_thrust_configuration_vector(self.alpha[1],self.r_th[1])
		t3 = calculate_thrust_configuration_vector(self.alpha[2],self.r_th[2])
		self.T = np.concatenate((t1,t2,t3),axis=1)

		self.T_inv = np.linalg.pinv(self.T)

		self.namespace = self.get_namespace()
		
		# if namespace is empty (i.e its '/'), then set it to unnamed_vessel
		if self.namespace == '/':
			self.namespace = 'unnamed_vessel'
		
		# Remove any slashes from the namespace
		self.namespace = self.namespace.replace('/','')

	def send_theoretically_controlled_effort(self):
		"""
		Send the theoretically controlled effort to the vehicle.
		"""

		# Make empty force vector of length 3x1
		f_thrusters = np.zeros(3)

		# Look up the forces per thruster
		for i in range(3):
			f_thrusters[i] = self.force_actuation_input_relation[i](self.u[i])

		# Calculate the theoretical control effort
		theoretical_control_effort = Wrench()
		tau = np.dot(self.T, f_thrusters)

		theoretical_control_effort.force.x = float(tau[0])
		theoretical_control_effort.force.y = float(tau[1])
		theoretical_control_effort.torque.z = float(tau[2])

		# Publish the message
		self.publisher_f_theoretical.publish(theoretical_control_effort)

		# Print the theoretical control effort
		#self.get_logger().info('Theoretical control effort: {}'.format(theoretical_control_effort))
		
	def check_allocation_requirements(self):
		"""
		Check if the required reference force and torque fields are not nan.
		Required fields are for surface operation:
		- self.reference.force.x
		- self.reference.force.y
		- self.reference.torque.z
		"""

		if np.isnan(self.reference.force.x) or np.isnan(self.reference.force.y) or np.isnan(self.reference.torque.z):
			return False
		else:
			return True

	# Overwrite the allocate_forces method
	def allocate_forces(self):
		""" According to fossen:
		- From thrust allocation matrix
		- From the desired force and torque
		
		- Calculate the thrusts per thruster


		Basic math here:
		tau = T * f
		f = T_inv * tau

		Where:
		tau is the force/torque vector
		f is the thrust vector
		T is the thrust allocation matrix
		"""

		# Create the force/torque vector
		tau = np.array([self.reference.force.x, self.reference.force.y, self.reference.torque.z])

		# Bound the force/torque vector to the allowed lower and upper limits
		for i in range(3):
			if tau[i] < self.force_allocation_limits[i][0]:
				tau[i] = self.force_allocation_limits[i][0]
			elif tau[i] > self.force_allocation_limits[i][1]:
				tau[i] = self.force_allocation_limits[i][1]

		# Calculate the force per thruster
		self.f_thrusters = np.dot(self.T_inv, tau)

		# initialize output actuation vector u as float32
		self.u = np.zeros(3, dtype=np.float32)

		# Look up actuation to satisfy the forces per actuator
		# assuming both f and self.actuator_force_input_relation are 3x1 vectors, fill each element of f in the actuator_force_input_relation
		for i in range(3):
			self.u[i] = self.actuator_force_input_relation[i](self.f_thrusters[i])
		
		# Check if the actuation is within the limits
		for i in range(3):
			if self.u[i] < self.thrust_limits[i][0]:
				self.u[i] = self.thrust_limits[i][0]
			elif self.u[i] > self.thrust_limits[i][1]:
				self.u[i] = self.thrust_limits[i][1]
		
		# Formulate ros message to send 
		msg = JointState()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.name = [	'SB_aft_thruster_propeller',
			  		 	'PS_aft_thruster_propeller', 
						'BOW_thruster_propeller',
						'SB_aft_thruster_joint',
						'PS_aft_thruster_joint'	]
		msg.position = [np.nan, np.nan, np.nan, self.alpha[0], self.alpha[1]]
		msg.velocity = [float(self.u[0]),float(self.u[1]),float(self.u[2]), np.nan, np.nan]
		msg.effort = [np.nan, np.nan, np.nan, np.nan, np.nan]

		# Publish the message
		self.publisher_actuation.publish(msg)

		# Send the theoretically controlled effort
		self.send_theoretically_controlled_effort()

def main(args=None):
	rclpy.init(args=args)

	node = ControlEffortAllocator_DP1()

	# Start the nodes processing thread
	rclpy.spin(node)

	# at termination of the code (generally with ctrl-c) Destroy the node explicitly
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
	