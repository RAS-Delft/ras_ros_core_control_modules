'''
    Starts the specified simulations representing the vessels in the fleet
'''

from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    ld = LaunchDescription()

    # Define the vesselids
    #vesselids = ['RAS_TN_DB','RAS_TN_RE','RAS_TN_GR','RAS_TN_OR','RAS_TN_LB','RAS_TN_YE','RAS_TN_PU']
    #vesselids = ['RAS_TN_DB','RAS_TN_OR','RAS_TN_GR']
    vesselids = ['RAS_TN_DB']
    
    # Launch vesselstack.launch.py for each vesselid
    for vesselid in vesselids:
        turtleboat_node = Node(
            package='turtleboat',
            executable='turtleboatmain',
            name='turtleboat_sim',
            namespace=vesselid,
            parameters=[{'rate_publish_position':20.0},
                        {'rate_publish_heading':17.0}],
            output='screen',
            emulate_tty=True,
        )
        ld.add_action(turtleboat_node)