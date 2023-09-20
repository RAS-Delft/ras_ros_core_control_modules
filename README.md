# ras-ros-core-control-modules
ras-ros-core-control-modules is a collection of core components specifically designed to serve as the foundational building blocks for implementing automated vessel control systems. Built upon the Robot Operating System (ROS), this repository encapsulates a suite of essential modules that enable the development of versatile and adaptable vessel control solutions developed in the Researchlab Autonomous Shipping (RAS) Delft.

Unlike repositories focused on project-specific software, ras-ros-core-control-modules emphasizes the creation of universally applicable tools that can be integrated into various maritime automation projects.

## Python modules:
- The **/lib** folder contains various python modules that contain functions to be used in various projects. Think of coordinate transformations, known model parameters to convenient python functions that are common in making control algorythms.
- The **/thrustAllocation** folder contains scripts or nodes that convert desired resultant forces to actuator reference (that are supposed to satisfy the desired effect)
- The **/motionController** folder contains mid-level controllers that attempt to let ship states follow a provided reference. Think of (1dof) heading controller, (2dof) heading + surge-velocity controller or a (3dof) dynamical positioning system.
- The **/sensorProcessing** folder contains scripts that process one or more sensor streams to get new or improved information. E.g. noise filtering or differentiating position to get a velocity.
- (more upcoming, e.g. Guidance folder with simple off the shelf guidance algorythms.)
