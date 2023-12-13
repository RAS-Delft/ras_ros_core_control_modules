"""
Launch file for starting up a fleet of vessels in a simulated environment, & peripherals

Components: 
- Simulator for each vessel
- geodetic to cartesian conversion for each vessel
- static sensor transforms (from sensor to base link) for each vessel
- Joint state publisher for the vessel's actuators (reference is published on tf tree, as we do not have measurements of the actuators) for each vessel
- actuator reference streaming on transform tree + robot urdf description publisher for each vessel
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import ras_ros_core_control_modules.tools.display_tools as ras_display_tools
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ld = LaunchDescription()

    # Define the vesselids
    #vesselids = ['RAS_TN_DB','RAS_TN_RE','RAS_TN_GR','RAS_TN_OR','RAS_TN_LB','RAS_TN_YE','RAS_TN_PU']
    vesselids = ['RAS_TN_DB','RAS_TN_OR','RAS_TN_GR']
    ras_urdf_common_path = FindPackageShare('ras_urdf_common')
    default_model_path = PathJoinSubstitution(['urdf', 'titoneri.urdf.xacro'])
    
    # Launch vesselstack.launch.py for each vesselid
    for vesselid in vesselids:

        # Make urdf description
        robot_description_content = None
        robot_description_content = ParameterValue(Command(['xacro ',
                                                        PathJoinSubstitution([ras_urdf_common_path, default_model_path]),
                                                        ' vesselcolor:="',ras_display_tools.vesselcolors_rgb.get_normalized_string(vesselid) + ' 0.9"',
                                                        ' vesselid:="',vesselid,'"']),
                                                        value_type=str)
    
        # Start robot description content publisher
        robot_state_publisher_node = Node(package='robot_state_publisher',
                                    executable='robot_state_publisher',
                                    namespace=vesselid,
                                    parameters=[{
                                          'robot_description': robot_description_content,
                                    }])
        ld.add_action(robot_state_publisher_node)
        
        # Start up various vessel control from vesselstack.launch.py, including simulator
        vesselnodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ get_package_share_directory('ras_ros_core_control_modules'),'/vessel_sim.launch.py']),
                launch_arguments={'vessel_id': vesselid,
                                  'vessel_color':ras_display_tools.vesselcolors_rgb.get_normalized_string(vesselid) + " 0.8"}.items()
            )
        ld.add_action(vesselnodes)

    return ld