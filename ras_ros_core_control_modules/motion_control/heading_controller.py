#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import argparse
from std_msgs.msg import Float32, Float32MultiArray
import ras_ros_core_control_modules.tools.display_tools as ras_display_tools
from ras_ros_core_control_modules.tools.control_tools import PIDController
import ras_ros_core_control_modules.tools.geometry_tools as ras_geometry_tools

# Get arguments
parser = argparse.ArgumentParser(description='ROS2 control module for heading control')
parser.add_argument("objectID", type=str,help="set vessel identifier")
parser.add_argument('-r') # ROS2 arguments
args, unknown = parser.parse_known_args()

# Set constants
OBJECT_ID = args.objectID
PERIOD_BROADCAST_STATUS = 5.0 # [s]

class PIDController_rotationally(PIDController):
	"""
	Subclass of PIDController with variation that calculates the error in a rotational sense. 
	This means that the error is calculated as the shortest angle between the reference and the state.
	"""
	def calc_error(self):
		"""
		Calculate the error in a rotational sense
		Overwrites the (linear) calc_error method of the PIDController class to be rotationally continuous between 0 and 2pi
		"""
		return ras_geometry_tools.signed_shortest_angle_radians(self.ref,self.state)

class HeadingControllerNode(Node):
	def __init__(self):
		super().__init__('heading_controller')
		
		# create PID controller
		self.headingPID = PIDController_rotationally(0.32,0.0,0.0)# ki = 0.0962
		self.headingPID.output_limits=[-0.63,0.63] # [N*m] NEW, referring to desired torque output
		self.headingPID.integral_limits = [-1.0,1.0]
		
		# Set up publishers and subscribers
		self.subscriber_reference = self.create_subscription(Float32,'reference/heading',self.callback_reference,10)
		self.subscriber_state = self.create_subscription(Float32,'telemetry/heading',self.callback_state,10)
		self.publisher_pid_status = self.create_publisher(Float32MultiArray,'diagnostics/headingController/pid_status',10)
		self.publisher_desired_torque = self.create_publisher(Float32,'reference/controlEffort/torqueZ',10)

		# Statistics
		self.timer_statistics = self.create_timer(PERIOD_BROADCAST_STATUS, self.print_statistics)
		self.timer_statistics_last = self.get_clock().now().nanoseconds/1e9
		self.tracker_callback_reference = 0
		self.tracker_callback_state = 0
		self.tracker_callback_control = 0
	
	def callback_reference(self,msg:Float32):
		self.tracker_callback_reference += 1
		self.headingPID.setRef(msg.data)
		self.run_controls()

	def callback_state(self,msg:Float32):
		self.tracker_callback_state += 1
		self.headingPID.setState(msg.data)
		self.run_controls()
	
	def run_controls(self):
		# Update tracker
		self.tracker_callback_control += 1

		# Run PID controller
		desired_torque = self.headingPID.compute() 

		# Publish actuation
		msg = Float32(data=desired_torque)
		self.publisher_desired_torque.publish(msg)

	def print_statistics(self):
		""" On a single line, print the rates of all major callbacks in this script. """

		# Calculate passed time
		now = self.get_clock().now().nanoseconds/1e9
		passed_time = now - self.timer_statistics_last

		# Calculate rates up to two decimals
		rate_reference = round(self.tracker_callback_reference/passed_time,2)
		rate_state = round(self.tracker_callback_state/passed_time,2)
		rate_control = round(self.tracker_callback_control/passed_time,2)

		# Format information to string
		printstring = ras_display_tools.terminal_fleet_module_string(self.get_namespace()[1:], ['reference_rate',rate_reference,'hz'],['state_rate',rate_state,'hz'],['control_rate',rate_control,'hz'])

		# Print
		self.get_logger().info(printstring)

		# Reset trackers
		self.tracker_callback_reference = 0
		self.tracker_callback_state = 0
		self.tracker_callback_control = 0
		self.timer_statistics_last = now

def main(args=None):
	rclpy.init(args=args)

	node = HeadingControllerNode()

	# Start the nodes processing thread
	rclpy.spin(node)

	# at termination of the code (generally with ctrl-c) Destroy the node explicitly
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()