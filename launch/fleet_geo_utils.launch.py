"""
Launch file for starting up a auxillary systems for outside operation of the fleet
General purpose for both simulation and real ships, to make data available for the rest of the system (e.g. transforms in tf tree & urdf publication for rviz, etc.)

Components that are not essential, but generally convenient or desired for both simulation and real ships in outside environment are grouped here. 
Control decision making agents are not intended in this launch file.


Components: 
- robot_state_publisher_node for each vessel doing the following:
    - Unified Robot Description Format publisher for each vessel
    - convert actuator joint state positions to tf tree for each vessel (real time, continuous)

- navsat_transform_node for each vessel doing the following:
    - convert geo-coordinates to cartesian coordinates on tf tree (real time, continuous)

- static_transform_node for each vessel doing the following:
    - convert (static) sensor positions to tf tree publication

 """

from launch import LaunchDescription
from launch_ros.actions import Node
import ras_ros_core_control_modules.tools.display_tools as ras_display_tools
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description(vesselids_list = ['RAS_TN_DB']):
    ld = LaunchDescription()

    print("vesselids_list: ",vesselids_list)
    # Define the vesselids
    #vesselids = ['RAS_TN_DB','RAS_TN_RE','RAS_TN_GR','RAS_TN_OR','RAS_TN_LB','RAS_TN_YE','RAS_TN_PU']
    vesselids = ['RAS_TN_DB']#,'RAS_TN_OR','RAS_TN_GR']
    ras_urdf_common_path = FindPackageShare('ras_urdf_common')
    default_model_path = PathJoinSubstitution(['urdf', 'titoneri.urdf.xacro'])
    
    # Launch vesselstack.launch.py for each vesselid
    for vesselid in vesselids:

        # Start robot description content publisher
        ## Make urdf description
        robot_description_content = None
        robot_description_content = ParameterValue(Command(['xacro ',
                                                        PathJoinSubstitution([ras_urdf_common_path, default_model_path]),
                                                        ' vesselcolor:="',ras_display_tools.vesselcolors_rgb.get_normalized_string(vesselid) + ' 0.9"',]),
                                                        value_type=str)
        
        ## Publish the robot description
        robot_state_publisher_node = Node(package='robot_state_publisher',
                                    executable='robot_state_publisher',
                                    namespace=vesselid,
                                    parameters=[{
                                          'robot_description': robot_description_content,
                                          'publish_frequency': 30.0,
                                          'frame_prefix': vesselid + '/',
                                    }],
                                    remappings=[
                                        ('joint_states', 'reference/actuation_prio'),
                                        ]
                                    )
        ld.add_action(robot_state_publisher_node)

        baselink_arg = vesselid+'/base_link'

        navsat_transform_node = Node(
            package='ras_ros_core_control_modules',
            executable='navsat_cartesian_transform',
            name='geo_cartesian_tf',
            namespace=vesselid,
            parameters=[{
                'datum':[52.00158915434494,4.371940750746264,0],
                'map_frame': 'world',
                'base_link_frame': baselink_arg
            }],
            output='screen',
            emulate_tty=True,
        )
        ld.add_action(navsat_transform_node)


        # placement of the sensors is a coarse measurement wrt the middle of the hatch - to be updated if ever need to be used accurately
        static_transform_node_gnss = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher_sensor_gnss',
                namespace=vesselid,
                arguments = ['-0.02', '-0.09', '-0.13', '0', '0', '0', baselink_arg, vesselid+'/gnss0'],
            )
        
        static_transform_node_imu = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher_sensor_imu',
                namespace=vesselid,
                arguments = ['0.155', '0.0', '-0.23', '0', '0', '0', baselink_arg, vesselid+'/imu0'],
            )
        
        static_transform_node_camera = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher_sensor_camera',
                namespace=vesselid,
                arguments = ['0.38', '0', '-0.09', '0', '0', '0', baselink_arg, vesselid+'/cam0'],
            )
        
        ld.add_action(static_transform_node_gnss)
        ld.add_action(static_transform_node_imu)
        ld.add_action(static_transform_node_camera)

    return ld