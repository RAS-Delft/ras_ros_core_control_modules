from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dummy_publisher_gui_bart_1',
            executable='demo1',
            namespace='pub1_with_joystick',
            name='ros2_manual_control_interface'
        )
    ])