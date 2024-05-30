from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(vesselids = ['RAS_TN_DB','RAS_TN_OR','RAS_TN_GR']):
    ld = LaunchDescription()
    vesselids = ['RAS_TN_DB','RAS_TN_OR','RAS_TN_GR','RAS_TN_YE','RAS_TN_RD','RAS_TN_LB','RAS_TN_PU']
    
    # Bring up the control stack for each vessel
    for vesselid in vesselids:
        turtleboat_node = Node(
            package='turtleboat',
            executable='turtleboatmain',
            name='turtleboat_sim',
            namespace=vesselid,
            output='screen',
            emulate_tty=True,
        )
        ld.add_action(turtleboat_node)
    return ld