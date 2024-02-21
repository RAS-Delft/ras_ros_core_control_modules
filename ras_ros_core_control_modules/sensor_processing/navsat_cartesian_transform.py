import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from tf2_ros import TransformBroadcaster, TransformStamped
import pyproj
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy

# import ras_ros_core_control_modules/tools/display_tools package  
import ras_ros_core_control_modules.tools.display_tools as display_tools

class NavsatCartesianTransformNode(Node):
    """
    This node converts two sensor messages, geo-coordinate & Imu orientation, referring to a robot's pose, to a transform message.
    The coordinate system is a cartesian coordinate system, with the origin at the datum (Optionally specified as argument).
    """

    def __init__(self):
        super().__init__('navsat2cartesian_transform')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('datum', [52.00, 4.37,0.0]),
				('map_frame', 'map'),
                ('base_link_frame', 'base_link'),
                ('period_broadcast_status', 5.0),
				]
        )

        self.base_link_frame = self.get_parameter('base_link_frame').get_parameter_value().string_value
        # Overwrite base_link_frame if it is set to 'base_link' AND the namespace is not empty (i.e. not the root namespace /)
        if self.get_parameter('base_link_frame').get_parameter_value().string_value == 'base_link' and self.get_namespace() != '/':
            self.base_link_frame = self.get_namespace()[1:] + '/base_link'     

        # Define the QoS profile for the publisher
        qos_profile_control_data = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.navsatfix_subscription = self.create_subscription(NavSatFix, 'telemetry/gnss/fix', self.gnss_callback, qos_profile_control_data)
        self.imu_subscription = self.create_subscription(Imu, 'telemetry/imu', self.imu_callback, qos_profile_control_data)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.latest_navsatfix = None
        self.latest_imu = None

        # Make a transformer helper object from lat/lon to x/y
        # Currently hardcoded to the Dutch RD projection EPSG:28992
        self.geo_cart_transformer = pyproj.Transformer.from_crs('epsg:4326', 'epsg:28992')
        datum = self.get_parameter('datum').get_parameter_value().double_array_value
        self.origin_correction = self.geo_cart_transformer.transform(datum[1], datum[0])

        # Statistics
        self.timer_statistics = self.create_timer(self.get_parameter('period_broadcast_status').get_parameter_value().double_value, self.print_statistics)
        self.timer_statistics_last = self.get_clock().now().nanoseconds/1e9
        self.tracker_imu = 0
        self.tracker_navsatfix = 0
        self.tracker_timer1 = 0

    def gnss_callback(self, msg):
        """
        Stores a received NavSatFix message in the latest_navsatfix variable.
        """
        self.tracker_navsatfix += 1
        self.latest_navsatfix = msg
        
    def imu_callback(self, msg):
        """
        Stores a received Imu message in the latest_imu variable.
        """
        self.tracker_imu += 1
        self.latest_imu = msg

    def timer_callback(self):
        """
        Transforms the latest NavSatFix and Imu messages to a transform message and publishes it.
        This function is called periodically by the timer.
        """
        self.tracker_timer1 += 1
        if self.latest_navsatfix is None or self.latest_imu is None:
            return
        
        # Convert latitude and longitude to local x and y coordinates
        lat = self.latest_navsatfix.latitude
        lon = self.latest_navsatfix.longitude
        x, y = self.geo_cart_transformer.transform(lon, lat)

        # Correct for origin
        x -= self.origin_correction[0]
        y -= self.origin_correction[1]

        # Create transform message
        q = self.latest_imu.orientation
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.get_parameter('map_frame').get_parameter_value().string_value
        transform.child_frame_id = self.base_link_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = q.x
        transform.transform.rotation.y = q.y
        transform.transform.rotation.z = q.z
        transform.transform.rotation.w = q.w

        # Publish transform message
        self.tf_broadcaster.sendTransform(transform)
    
    def print_statistics(self):
        """ On a single line, print the rates of all major callbacks in this script. """

        # Calculate passed time
        now = self.get_clock().now().nanoseconds/1e9
        passed_time = now - self.timer_statistics_last

        # Calculate rates   
        rate_navsatfix = self.tracker_navsatfix / passed_time
        rate_imu = self.tracker_imu / passed_time
        rate_timer1 = self.tracker_timer1 / passed_time

		# Format information to string
        printstring = display_tools.terminal_fleet_module_string(self.get_namespace()[1:],['gnss_rate',rate_navsatfix,'hz'],['imu_rate',rate_imu,'hz'],['send_transform_rate',rate_timer1,'hz'])

        # Print
        self.get_logger().info(printstring)

        # Reset trackers
        self.tracker_navsatfix = 0
        self.tracker_imu = 0
        self.tracker_timer1 = 0
        self.timer_statistics_last = now

def main(args=None):
	rclpy.init(args=args)

	node = NavsatCartesianTransformNode()

	# Start the nodes processing thread
	rclpy.spin(node)

	# at termination of the code (generally with ctrl-c) Destroy the node explicitly
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
	