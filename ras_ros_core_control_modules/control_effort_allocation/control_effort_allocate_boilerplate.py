'''
This control effort allocator subclasses control_effort_allocator_super.py and overrides the allocate_forces method to implement a simple control effort allocation protocol.
'''

## Import the superclass
from ras_ros_core_control_modules.control_effort_allocation.control_effort_allocator_super import ControlEffortAllocatorNode


class ControlEffortAllocator_DP1(ControlEffortAllocatorNode):
	""" 
	ROS2 control module for allocation of control efforts to actuators.
	This particular version has fixed angles of the thrusters and uses simple matrix multiplication to figure out thrust per actuator and sends this
	"""
	def __init__(self):
		super().__init__('dp_control_effort_allocator')

	# Overwrite the allocate_forces method
	def allocate_forces(self):
		""" According to fossen:
		- From thrust allocation matrix
		- From the desired force and torque
		
		- Calculate the thrusts per thruster
		"""
		print("Allocate forces according to DP1 protocol")
		pass
