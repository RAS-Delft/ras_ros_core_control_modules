"""
    Launch file that brings up major components to set up a simulated vessel

    This launch file is commonly called from a higher level fleet launch file, as this is only for one ship. 

    Components:
    - Simulator (turtleboat)
    - robot description publisher
    - static sensor transforms (from sensor to base link)
    - Conversion from geodetic (gnss) to cartesian (local) coordinates (navsat_cartesian_transform)
    - Joint state publisher for the vessel's actuators (reference is published on tf tree, as we do not have measurements of the actuators)
"""


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Load and/or set the parameters -----------------------------------------

    # Define the arguments for the launch file
    vessel_id_arg = DeclareLaunchArgument(
        'vessel_id',
        default_value='default_vessel',
        description='ID of the vessel'
    )
    vessel_id_value = LaunchConfiguration('vessel_id')
    ld.add_action(vessel_id_arg)

    starting_position_arg = DeclareLaunchArgument(
        'starting_position',
        default_value='52.0015288045564,4.37201677404399,0.0,0.0,0.0,0.0',
        description='Starting position of the vessel'
    )
    starting_position_value = LaunchConfiguration('starting_position')
    ld.add_action(starting_position_arg)

    # Define the nodes --------------------------------------------------------

    navsat_transform_node = Node(
        package='ras_ros_core_control_modules',
        executable='navsat_cartesian_transform',
        name='geo_cartesian_tf',
        namespace=vessel_id_value,
        arguments=[vessel_id_value,'-d','52.00158915434494','4.371940750746264','0','-p','world','-c',[vessel_id_value,'/base_link']],
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(navsat_transform_node)
    
    turtleboat_node = Node(
        package='turtleboat',
        executable='turtleboatmain',
        name='turtleboat_sim',
        namespace=vessel_id_value,
        arguments=[vessel_id_value,'-imu','true','-saux','false','-p',starting_position_value,'-rpos','20'],
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(turtleboat_node)

    # Set up the joint state publisher node -----------------------------------
    joint_state_publisher_node = Node(package='ras_urdf_common',
                                    executable='azi_joint_state_broadcaster.py',
                                    namespace=vessel_id_value,
                                    arguments=[vessel_id_value,'-t','tito neri'],
                                    output='screen',
                                    emulate_tty=True,
                                    
                                    )
    ld.add_action(joint_state_publisher_node)

    # Set up static transforms for the vessel's sensors -----------------------
    baselink_arg = [LaunchConfiguration('vessel_id'), '/base_link']

    # placement of the sensors is a coarse measurement wrt the middle of the hatch - to be updated if ever need to be used accurately
    static_transform_node_gnss = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_sensor_gnss',
            namespace=vessel_id_value,
            arguments = ['-0.02', '-0.09', '-0.13', '0', '0', '0', baselink_arg, [LaunchConfiguration('vessel_id'), '/gnss0']],
        )
    
    static_transform_node_imu = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_sensor_imu',
            namespace=vessel_id_value,
            arguments = ['0.155', '0.0', '-0.23', '0', '0', '0', baselink_arg, [LaunchConfiguration('vessel_id'), '/imu0']],
        )
    
    static_transform_node_camera = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_sensor_camera',
            namespace=vessel_id_value,
            arguments = ['0.38', '0', '-0.09', '0', '0', '0', baselink_arg, [LaunchConfiguration('vessel_id'), '/cam0']],
        )
    ld.add_action(static_transform_node_gnss)
    ld.add_action(static_transform_node_imu)
    ld.add_action(static_transform_node_camera)
    return ld