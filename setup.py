from setuptools import setup

package_name = 'ras_ros_core_control_modules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[


        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/fleet_sim.launch.py','launch/vessel_sim.launch.py']),

        ## BB I have no clue why python wants the items to be installed in this specific location.
        ## All I know is that if installed in this location, other ros packages can find them.
        # I guess it is just moving things from source to install space, although (specifically) the python files themselves do not need building. 
        ('lib/python3.8/site-packages/' + package_name + '/tools', [package_name+'/tools/geometry_tools.py',
                                                                    package_name+'/tools/titoneri_parameters.py',
                                                                    package_name+'/tools/display_tools.py',
                                                                    package_name+'/tools/control_tools.py']),
        ('lib/python3.8/site-packages/' + package_name + '/sensor_processing', [package_name+'/sensor_processing/vel_differentiator.py',package_name+'/sensor_processing/navsat_cartesian_transform.py']),
        ('lib/python3.8/site-packages/' + package_name + '/motion_control', [package_name+'/motion_control/heading_controller.py', 
                                                                             package_name+'/motion_control/surge_vel_controller.py']),
        ('lib/python3.8/site-packages/' + package_name + '/control_effort_allocation', [package_name+'/control_effort_allocation/TN_control_effort_allocator_nomoto.py']),
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
        ],
    },
)
