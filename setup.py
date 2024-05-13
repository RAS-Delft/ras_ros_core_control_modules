from setuptools import setup
import os
from glob import glob

package_name = 'ras_ros_core_control_modules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name+'/sensor_processing',package_name+'/motion_control',package_name+'/control_effort_allocation',package_name+'/tools'],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/fleet_sim.launch.py','launch/fleet_geo_utils.launch.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bart',
    maintainer_email='bartboogmans@hotmail.com',
    description='Core control modules of Researchlab Autonomous Shipping, Delft',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navsat_cartesian_transform = ras_ros_core_control_modules.sensor_processing.navsat_cartesian_transform:main',
            'vel_differentiator = ras_ros_core_control_modules.sensor_processing.vel_differentiator:main',
            'heading_controller = ras_ros_core_control_modules.motion_control.heading_controller:main',
            'surge_vel_controller = ras_ros_core_control_modules.motion_control.surge_vel_controller:main',
            'TN_control_effort_allocator_nomoto = ras_ros_core_control_modules.control_effort_allocation.TN_control_effort_allocator_nomoto:main',
            'TN_control_effort_allocator_dp1 = ras_ros_core_control_modules.control_effort_allocation.TN_control_effort_allocator_dp_01:main',
        ],
    },
)
