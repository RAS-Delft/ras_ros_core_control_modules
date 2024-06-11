#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

from ras_ros_core_control_modules.tools.control_tools import PIDController
import ras_ros_core_control_modules.tools.display_tools as ras_display_tools

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class SurgeVelControllerNode(Node):
    def __init__(self):
        super().__init__('surge_vel_controller')
        custom_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Set up ros parameters
        self.declare_parameter('period_control',1.0/16.0)
        self.declare_parameter('period_broadcast_status',5.0)
        self.declare_parameter('enable_report_pid_status',False)

        # Make PID controllers
        self.PID = PIDController(21.0,0.0,0.0) # kp was 21.0 kd was 2.55
        self.PID.integral_limits=[-8*0.25, 8*0.25] # set limit to a total buildup of 8 seconds with a characteristic error of 0.25m/s/s
        self.PID.output_limits=[0.1,3.0] # Newtons of thrust from both aft thrusters combined

        # Set up publishers and subscribers
        self.publisher_forceX = self.create_publisher(Float32,'reference/controlEffort/forceX',custom_qos_profile)
        self.subscriber_reference = self.create_subscription(Float32MultiArray,'reference/velocity',self.reference_callback,custom_qos_profile)
        self.subscriber_state = self.create_subscription(Float32MultiArray,'state/velocity',self.state_callback,custom_qos_profile)
        
        if self.get_parameter('enable_report_pid_status').value:
            self.publisher_pid_status = self.create_publisher(Float32MultiArray,'diagnostics/surgeVelocityController/pid_status',custom_qos_profile)
        else:
            self.publisher_pid_status = None

        # Set up timer for control loop
        self.timer_control = self.create_timer(self.get_parameter('period_control').get_parameter_value().double_value, self.run_controls)

        # Set up statistics
        self.timer_statistics = self.create_timer(self.get_parameter('period_broadcast_status').get_parameter_value().double_value, self.print_statistics)
        self.timer_statistics_last = self.get_clock().now().nanoseconds/1e9
        self.tracker_callback_reference = 0
        self.tracker_callback_state = 0
        self.tracker_callback_control = 0

    def reference_callback(self, msg:Float32MultiArray):
        self.tracker_callback_reference += 1
        self.PID.setRef(msg.data[0])

    def state_callback(self, msg:Float32MultiArray):
        self.tracker_callback_state += 1
        self.PID.setState(msg.data[0])
    
    def run_controls(self):
        # add to rate tracker
        self.tracker_callback_control += 1
        
        # Calculate PID output
        out = self.PID.compute()

        # Send message
        msg = Float32(data=out)
        self.publisher_forceX.publish(msg)

    def print_statistics(self):
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

    node = SurgeVelControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(" Stopping cleanly")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
    