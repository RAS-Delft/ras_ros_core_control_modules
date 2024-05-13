# ras_ros_core_control_modules
ras_ros_core_control_modules is a collection of core components specifically designed to serve as the foundational building blocks for implementing automated vessel control systems. Built upon the Robot Operating System (ROS), this repository encapsulates a suite of essential modules that enable the development of versatile and adaptable vessel control solutions developed in the Researchlab Autonomous Shipping (RAS) Delft.

Unlike repositories focused on project-specific software, ras-ros-core-control-modules emphasizes the creation of universally applicable tools that can be integrated into various maritime automation projects.

## Module groups
For purposes of modularity we try to group protocols in a specific group, if possible. The image below shows a typical marine control structure, where blocks may be represented by 1 or more software modules/nodes that fulfill the function. 
![schematic-block-diagram-GCSF drawio](https://github.com/RAS-Delft/ras-ros-core-control-modules/assets/5917472/500c81e5-abe5-464f-97a8-3f27676f023a) <br>
Note that one function group depicted as a block can have a wide range of complexities. The function can be done by a script with simple functionalities, or be run by a whole network of modules by itself, where it's structure and complexity can vary according to the project goal. 

## Python module groups in this repository:
- The **/control-effort-allocation** folder contains scripts or nodes that convert desired resultant forces to actuator reference (that are supposed to satisfy the desired effect)
- The **/motion-control** folder contains mid-level controllers that attempt to let ship states follow a provided reference. Think of (1dof) heading controller, (2dof) heading + surge-velocity controller or a (3dof) dynamical positioning system.
- The **/sensor-processing** folder contains scripts that process one or more sensor streams to get new or improved information. E.g. noise filtering or differentiating position to get a velocity.
- (more upcoming, e.g. Guidance folder with simple off the shelf guidance algorythms.)
- The **/guidance** Folder contains software modules that end up generating motion control reference, which can include but is not limited to: pathfinding, situational decision makers, trajectory planning, logistical decision making, communication/negotiation with other agents. A simple guidance protocol can provide the motion control system with current desired state based on a predefined user given trajectory.
- The **/lib** folder contains various python modules that contain functions to be used in various projects. Think of coordinate transformations, known model parameters to convenient python functions that are common in making control algorythms.



## Requirements:
Pyproj:
```
pip install pyproj
```

Xacro
```
sudo apt-get update; sudo apt install ros-${ROS_DISTRO}-xacro
```