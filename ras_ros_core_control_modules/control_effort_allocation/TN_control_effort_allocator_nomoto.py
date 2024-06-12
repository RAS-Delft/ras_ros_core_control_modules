#!/usr/bin/env python

import rclpy
import time
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from rclpy.node import Node
import ras_ros_core_control_modules.tools.titoneri_parameters as titoneri_parameters
import ras_ros_core_control_modules.tools.display_tools as ras_display_tools

class ControlEffortAllocatorNode(Node):
    """ 
    ROS2 control module for allocation of control efforts to actuators. This particular version allocates x force and z torque, ignoring others dimentions. This is usable for having a ship doing heading+speed control, but not sway-control.
    """

    def __init__(self):
        super().__init__('nomoto_control_effort_allocator')
        custom_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Get vessel ID
        self.object_id = self.get_namespace()[1:]

        # Set up ros parameters
        self.declare_parameter('period_control',1.0/16.0)
        self.declare_parameter('period_broadcast_status',5.0)



        # Get relations between actuation and forces of all thrusters
        self.forceToThrust = titoneri_parameters.namespace_to_force2thrusterusage_array(self.object_id)

        # Set up publishers and subscribers
        self.publisher_actuation = self.create_publisher(JointState,'reference/actuation',custom_qos_profile)
        self.subscriber_torque = self.create_subscription(Float32,'reference/controlEffort/torqueZ',self.callback_torque,custom_qos_profile)
        self.subscriber_force_surge = self.create_subscription(Float32,'reference/controlEffort/forceX',self.callback_force_surge,custom_qos_profile)

        # Statistics
        self.timer_statistics = self.create_timer(self.get_parameter('period_broadcast_status').get_parameter_value().double_value, self.print_statistics)
        self.timer_statistics_last = self.get_clock().now().nanoseconds/1e9
        self.tracker_callback_torque = 0
        self.tracker_callback_force_surge = 0
        self.tracker_callback_run_allocationProtocol = 0
        self.torque = 0.0
        self.force_surge = 0.0

        self.timestamp_allocateF_last_allocate = time.time()

    def callback_torque(self,msg:Float32):
        self.tracker_callback_torque += 1
        self.torque = msg.data
        self.torque_set = 1
        self.allocate_forces()

    def callback_force_surge(self,msg:Float32):
        self.tracker_callback_force_surge += 1
        self.force_surge = msg.data
        self.force_surge_set = 1
        self.allocate_forces()
    
    def allocate_forces(self):
        # run controls if the last control was more than MIN_PERIOD_CONTROL seconds ago
        if time.time()-self.timestamp_allocateF_last_allocate> self.get_parameter('period_control').get_parameter_value().double_value:
            self.tracker_callback_run_allocationProtocol += 1
            # The two thrusters are treated as one thruster with two times the force at identical angles.
            # The location of the thruster with respect to the center origin of the vessel (in meters, x direction)
            r_thr = -0.42
            
            # maximum thrust output of each regular aft thruster
            max_thrust = 2.5 # N

            # Calculate the resultant force in x direction
            force_x = self.force_surge

            # Calculate the resultant force in y direction that the thruster needs to give to yield the desired torque
            force_y = self.torque/r_thr
            
            # Calculate the absolute force and angle of the resultant force per thruster
            force_abs = math.sqrt(force_x**2 + force_y**2)/2
            if force_abs == 0:
                force_angle = 0
            else:
                force_angle = math.atan2(force_y,force_x)
            
            # limit force_abs to max_thrust
            if force_abs > max_thrust:
                force_abs = max_thrust  
            
            # force per aft thruster is looked up from previously determined relation between force and thruster usage in rpm
            aft_propeller_vel_portside = self.forceToThrust[0](force_abs)*60
            aft_propeller_vel_starboard = self.forceToThrust[1](force_abs)*60

            # Publish actuation
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['SB_aft_thruster_propeller','PS_aft_thruster_propeller','BOW_thruster_propeller','SB_aft_thruster_joint','PS_aft_thruster_joint']
            msg.velocity = [aft_propeller_vel_starboard,aft_propeller_vel_portside,0.0,0.0]
            msg.position = [0.0,0.0,0.0, force_angle, force_angle]
            msg.effort = []

            self.publisher_actuation.publish(msg)
            
            self.timestamp_allocateF_last_allocate = time.time()

    def print_statistics(self):
        """
        On a single line, print the rates of all major callbacks in this script.
        """

        # Calculate passed time
        now = self.get_clock().now().nanoseconds/1e9
        passed_time = now - self.timer_statistics_last

        # Calculate rates up to two decimals
        rate_torque = round(self.tracker_callback_torque/passed_time,2)
        rate_force_surge = round(self.tracker_callback_force_surge/passed_time,2)
        rate_run_allocationProtocol = round(self.tracker_callback_run_allocationProtocol/passed_time,2)

        # Format information to string
        printstring = ras_display_tools.terminal_fleet_module_string(self.get_namespace()[1:], ['torque_rate',rate_torque,'hz'],['force_surge_rate',rate_force_surge,'hz'],['run_allocationProtocol_rate',rate_run_allocationProtocol,'hz'])

        # Print
        self.get_logger().info(printstring)

        # Reset trackers
        self.tracker_callback_torque = 0
        self.tracker_callback_force_surge = 0
        self.tracker_callback_run_allocationProtocol = 0
        self.timer_statistics_last = now

def main(args=None):
    rclpy.init(args=args)

    node = ControlEffortAllocatorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    