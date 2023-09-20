from setuptools import setup

package_name = 'ras-ros-core-control-modules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [package_name+'lib/control_tools.py']),
        ('share/' + package_name, [package_name+'lib/geometry_tools.py']),
        ('share/' + package_name, [package_name+'lib/display_tools.py']),
        ('share/' + package_name, [package_name+'lib/titoneri_parameters.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bart',
    maintainer_email='bartboogmans@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
