#!/usr/bin/env python

import rclpy
import time
import numpy as np
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Wrench
from rclpy.node import Node
import ras_ros_core_control_modules.tools.display_tools as ras_display_tools

class ControlEffortAllocatorNode(Node):
	""" 
	ROS2 control module for allocation of control efforts to actuators. This particular version allocates x force and z torque, ignoring others dimentions. This is usable for having a ship doing heading+speed control, but not sway-control.
	"""

	def __init__(self,forceToThrust_lookup_):
		super().__init__('control_effort_allocator')
		
		self.declare_parameters(
            namespace='',
            parameters=[
                ('status_broadcast_period', 5.0),
				]
        )

		# Get relations between actuation and forces of all thrusters
		self.forceToThrust = forceToThrust_lookup_

		# Set up publishers and subscribers
		self.publisher_actuation = self.create_publisher(JointState,'reference/actuation',10)
		self.subscriber_f = self.create_subscription(Wrench,'reference/control_effort',self.callback_control_effort,10)
		self.publisher_f_theoretical = self.create_publisher(Wrench,'reference/control_effort_theoretical',10)

		# Statistics
		self.timer_statistics = self.create_timer(self.get_parameter("status_broadcast_period").get_parameter_value().double_value, self.print_statistics)
		self.timer_statistics_last = self.get_clock().now().nanoseconds/1e9
		self.tracker_callback_control_effort = 0
		self.tracker_callback_run_allocationProtocol = 0

		self.reference = Wrench()
		# Put all contents to nan, as they are not know yet
		self.reference.force.x = np.nan
		self.reference.force.y = np.nan
		self.reference.force.z = np.nan
		self.reference.torque.x = np.nan
		self.reference.torque.y = np.nan
		self.reference.torque.z = np.nan

		self.timestamp_allocateF_last_allocate = time.time()


	def check_allocation_requirements(self):
		"""
        Checks constraints on the inputs to the allocation protocol.
		This function is a boilerplate for this superclass to be overridden by its subclasses.
		Generally this function checks if the required reference force and torque fields are not nan.
		Should return boolean True if the requirements are met, and False if they are not met.
		"""
		return True

	def allocate_with_check(self):
		"""
		Allocate forces to thrusters, but only if the requirements are met.
		"""
		if self.check_allocation_requirements():
			self.tracker_callback_run_allocationProtocol += 1
			self.allocate_forces()
		

	def callback_control_effort(self,msg:Wrench):
		self.tracker_callback_control_effort += 1

		# Store the control effort if the values are not nan
		if not np.isnan(msg.force.x):
			self.reference.force.x = msg.force.x
		if not np.isnan(msg.force.y):
			self.reference.force.y = msg.force.y
		if not np.isnan(msg.force.z):
			self.reference.force.z = msg.force.z
		if not np.isnan(msg.torque.x):
			self.reference.torque.x = msg.torque.x
		if not np.isnan(msg.torque.y):
			self.reference.torque.y = msg.torque.y
		if not np.isnan(msg.torque.z):
			self.reference.torque.z = msg.torque.z

		
		self.allocate_with_check()

	def allocate_forces(self):
		""" 
		Allocate forces to thrusters. 
		This is a boilerplate for this superclass to be overridden by its subclasses.

		This function should get self.reference Wrench object and attempt to allocate this over its actuators
		"""

		self.get_logger().warn('Allocate forces according to superclass protocol - this should be overridden by subclass')
			





	def print_statistics(self):
		"""
		On a single line, print the rates of all major callbacks in this script.
		"""

		# Calculate passed time
		now = self.get_clock().now().nanoseconds/1e9
		passed_time = now - self.timer_statistics_last

		# Calculate rates up to two decimals
		rate_callback_control_effort = round(self.tracker_callback_control_effort/passed_time,2)

		rate_run_allocationProtocol = round(self.tracker_callback_run_allocationProtocol/passed_time,2)

		# Format information to string
		printstring = ras_display_tools.terminal_fleet_module_string(self.get_namespace()[1:], ['control_effort_rate',rate_callback_control_effort,'hz'],['run_allocationProtocol_rate',rate_run_allocationProtocol,'hz'])

		# Print
		self.get_logger().info(printstring)

		# Reset trackers
		self.tracker_callback_control_effort = 0
		self.tracker_callback_force_surge = 0
		self.tracker_callback_run_allocationProtocol = 0
		self.timer_statistics_last = now

def main(args=None):
	rclpy.init(args=args)

	node = ControlEffortAllocatorNode()

	# Start the nodes processing thread
	rclpy.spin(node)

	# at termination of the code (generally with ctrl-c) Destroy the node explicitly
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
	