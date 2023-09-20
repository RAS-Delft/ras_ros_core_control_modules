""" This file contains the parameters of the Titoneri vessel.

        Note on the vessel coordinate system (Center-origin):
        The center origin is currently set to coincide with center gravity, although it can change due to loading.

        Some benchmarks: 
        - (current parameterisation in this file) CG is 510mm away from the aft when the ship has an intel NUC mounted under the hood.
        - CG is 500mm away from aft when the ship has no intel NUC mounted under the hood. (1 cm backwards w.r.t. above)

        The coordinate system used is North-East-Down.
"""

import numpy as np

""" Physical parameters """

def thruster_locations():
    """ Returns the position of the thrusters in meters with respect to the vessel's center-origin.
    1st column: Starboard azimuth thruster
    2nd column: Port azimuth thruster
    3rd column: Bow thruster

    1st row: x-coordinate
    2nd row: y-coordinate
    """    
    return np.array([[-.405,0.075],[-.405,-0.075],[0.311,0]])

def center_of_gravity():
    """ Returns the position of center of gravity of the vessel in meters with respect to the vessel's center-origin. """
    return np.array([[0.0],[0.0]])

def angles_zero():
    """ Returns the actuator angles at default position. (aft facing backwards, bow thruster sideways)
            1st item referring to: Starboard azimuth thruster
            2nd item referring to: Port azimuth thruster
            3rd item referring to: Bow thruster
        Units: Radians  
    """
    return np.array([0,0,np.pi/2])

def actuator_input_limits():
    """ Returns the lower and upper limits of the actuator inputs.
        Units:
            1st item: SB: RPS
            2nd item: PB: RPS
            3rd item: Bow: normalized pulse width modulation [-1:1]
    """
    return np.array([[-60,60],[-60,60],[-1.0,1.0]])

def actuator_orientation_limits():
    """ Returns the lower and upper limits of the actuator angles.

    Units:
        1st item: SB: rad
        2nd item: PB: rad
        3rd item: Bow: rad (note that this is fixed)
    """
    return np.array([[-np.pi*3/4,np.pi*3/4],[-np.pi*3/4,np.pi*3/4],[np.pi/2,np.pi/2]])

def actuator_input_rate_limits():
    """Returns the maximum rate of change of the actuator outputs.
    
    Units:
        1st item: rate_SB: rounds/s/s
        2nd item: rate_PS: rounds/s/s
        3rd item: rate_bow: 1/s (normalized pwm / second)
    """
    return np.array([120,120,2.0])

def actuator_orientation_rate_limits():
    """Returns the maximum rate of change of the actuator angles.
    
    Units:
        1st item: rate_SB: rad/s
        2nd item: rate_PS: rad/s
        3rd item: rate_bow: rad/s (fixed)
    """
    return np.array([np.pi*0.70,np.pi*0.70,0.0])

""" Thruster relationships
The following two functions give the relation between the thruster use and the force generated by the thruster of various types. 
The red thrusters are weaker than the black ones, hence the different relation. The relation of the red ship is currently unkown exactly, thus the 0.8 factor is a guess.
Aft thrusters have a second order polynomial relation here, as it is easily inverted to get the inverse relations. The bow thruster has a linear relation.
The aft thruster relationship is mirrorded in the negative direction, assuming it behaves similarly in backwards direction.

The relation of the black aft thrusters is based on a 2nd order polynomial fit of the data from technical report:
"Tito Neri model scale vessel thruster behaviour" March 17, 2022 - RAS Delft

A sidenote if anyone is interested in the data that resulted in the polynomial fit:
The dataset upon the curve is fitted is:
rps        force[N]
-----------------
17.0000    0.3254
25.0000    0.7080
27.0000    0.6981
28.0000    0.6883
    0    0.0899
28.0000    0.8257
12.0000    0.1095
    0    0.0409
32.0000    1.0611
37.0000    1.4829
25.0000    0.6000
39.0000    1.5320
34.0000    1.2867
    0    0.0605
22.0000    0.4529
26.0000    0.6099
    0    0.0213
31.0000    0.8453
21.0000    0.4038
38.0000    1.4535
7.0000    0.0114
39.0000    1.3358
27.0000    0.6883
33.0000    0.9924
8.0000   -0.0082
18.0000    0.2076
16.0000    0.1095
    0   -0.1063
    0   -0.1063
24.0000    0.3940
"""

def TN_thrusterusage_to_force1(thrustertype:str):
    """
    Gives the relation between the thruster use and the force generated
    The user is responsible of taking into account the limits of the thruster
    Aft_thrusters: quadratic relation
    Bow_thruster: linear relation

    Rough measurements of the black and red thrusters compared done by bartboogmans 
    at june 2023 showed that the red thrusters have a similarly shaped relation as the black thrusters,
    but with a factor of ~0.6584 with respect to the relation of the black model.

    Units:
        Aft thruster use: RPS
        Bow thruster use: PWM normalized [-1:1]
    """
    factor_red_thruster = 0.6584

    if thrustertype == "aft_black":
        return lambda v: (0.0009752*v*v)*np.sign(v)
    elif thrustertype == "aft_red":
        return lambda v: (0.0009752*v*v*factor_red_thruster)*np.sign(v)
    elif thrustertype == "bowThruster":
        return lambda PWM_value: PWM_value*3.575

def TN_force_to_thrusterusage1(thrustertype:str):
    """
    Gives the relation between the force generated and the thruster use
    The user is responsible of taking into account the limits of the thruster
    Aft_thrusters: quadratic relation
    Bow_thruster: linear relation

    Rough measurements of the black and red thrusters compared done by bartboogmans 
    at june 2023 showed that the red thrusters have a similarly shaped relation as the black thrusters,
    but with a factor of ~0.6584 with respect to the relation of the black model.

    Units:
        Aft thruster use: RPS
        Bow thruster use: PWM normalized [-1:1]
    """
    factor_red_thruster = 0.6584

    if thrustertype == "aft_black":
        return lambda f: np.sqrt(f/0.0009752)*np.sign(f)
    elif thrustertype == "aft_red":
        return lambda f: np.sqrt(f/(0.0009752*factor_red_thruster))*np.sign(f)    
    elif thrustertype == "bowThruster":
        return lambda f: f/3.575

def TN_thrusterusage_to_force2(thrustertype:str):
    """
    This function is the old relationship of a third order polynomial fit.
    Evenly numbered orders are not considered to make the fit represent backwards motion as well, as it will be symmetric.
    Unfortunately this third order fit is not so easily inverted, hence the new relation above.
    Aft_thrusters: ax^3+bx relation (no quadratic term)
    Bow_thruster: linear relation
    
    Approximated by bartboogmans ~ august 2020

    Factor for red thruster type approximated by bartboogmans ~ june 2023
    """
    factor_red_thruster = 0.6584
    
    if thrustertype == "aft_black":
        return lambda v: ((1.925e-5)*v*v*v+(1.061e-2)*v)
    elif thrustertype == "aft_red":
        return lambda v: ((1.925e-5)*v*v*v+(1.061e-2)*v)*factor_red_thruster
    elif thrustertype == "bowThruster":
        return lambda PWM_value: PWM_value*3.575


""" State space model parameters """

def model_Mrb():
    """ Returns the rigid body inertial matrix"""
    # [BB] Note that the current diagonals on inertia in pitch and roll direction are 1. 
    # These values are not measured (yet), although the space is allocated for 6DOF dynamics
    # These points have to be nonzero to make this matrix invertible. 
    # To not have any (incorrect) dynamics in these directions, simply make sure that no forces are applied in these directions.

    # Since this matrix is diagonal, this must mean that this matrix is defined with Center-origin=Center of gravity.
    return np.array([ 
        [16.9    ,0        ,0        ,0        ,0        ,0        ],
        [0        ,16.9    ,0        ,0        ,0        ,0        ],
        [0        ,0        ,16.9    ,0        ,0        ,0        ],
        [0        ,0        ,0        ,1        ,0        ,0        ],
        [0        ,0        ,0        ,0        ,1        ,0        ],
        [0        ,0        ,0        ,0        ,0        ,0.51   ]
    ])

def model_Ma():
    """Returns the inertial matrix of the hydrodynamic added mass"""
    return np.array([        
        [1.2    ,0        ,0        ,0        ,0        ,0        ],
        [0        ,49.2    ,0        ,0        ,0        ,0        ],
        [0        ,0        ,0        ,0        ,0        ,0        ],
        [0        ,0        ,0        ,0        ,0        ,0        ],
        [0        ,0        ,0        ,0        ,0        ,0        ],
        [0        ,0        ,0        ,0        ,0        ,1.8    ]
    ])

def model_D():
    """ Returns the linear damping matrix"""
    
    """ Some notes on the dampening coefficients here: [BB]
    1) The origonal matrices that have been published in RAS related articles using TitoNeri models 
    did not have off diagonal elements. 
        We know that this is not correct, as the hull is non-symmetric in the Y direction. (sideways)
        We do, however know, that the ship is symmetrical in X direction. (forward)
        
        We also know that the ship has natural tendencies to steer it's bow towards the direction of 
        motion, giving information of the sign of this element. 
        
        This information resulted in a rough approximation of the off diagonal elements at [5,1] and [1,5].
        It is based on the assumption that forces due to motion in the Y direction are a little bit (5cm) shifted
        to the back of the ship, resulting in a bit of torque, and vice versa. 
        
        The absolute value of this coupling can be measured properly in the future, but at least 
        some of these dynamics are now included in the right direction.
        
    2) The damping effects are not linear, although this dampening matrix is used as if it is.
    
        This was particularly noticable when model rotation dynamics were compared with the real ship.
        
        Parameters were iteratively tuned to match feeling of the dynamics at that point. 
        But when the dampening felt sensible for small turning rates, the ship would dampen too little for 
        large turning rates and inverse as well. This is due to the fact that the dampening is actually not linear. 

        A short term solution is scaling the dampening factors to match the working range of a particular experiment. 
        
        A long term objective is to find good approximations of these dynamics (e.g. quadratic?).

        For example:    At high rotational velocities the yaw dampening would be ~ 0.525    (Nm/(rad/s))
                        At low rotational velocities the yaw dampening would be ~ 0.525*0.20 (Nm/(rad/s))
    """
    return np.array([         
        [2.6416     ,0        ,0        ,0        ,0        ,0            ],
        [0        ,21.9034,0        ,0        ,0        ,-1.0952    ],
        [0        ,0        ,0        ,0        ,0        ,0            ],
        [0        ,0        ,0        ,0        ,0        ,0            ],
        [0        ,0        ,0        ,0        ,0        ,0            ],
        [0        ,-1.0952,0        ,0        ,0        ,0.525*0.20    ]
    ])

""" Visuals """

def vessel_outline():
    """Returns a 2d, topview, vessel outline in the form of a numpy array. """

    return np.array([
            [-0.105,-0.105,-0.0525,0.60,0.65,0.70,0.80,0.85,0.88,0.88,0.85,0.80,0.70,0.65,0.60,-0.0525,-0.105], \
            [-0.11,0.11,0.1625,0.1625,0.1575,0.145,0.1025,0.07,0.04,-0.04,-0.07,-0.1025,-0.145,-0.1575,-0.1625,-0.1625,-0.11]]) \
            + np.array([[-0.405],[0.0]]) # offset wrt azimuth thruster positioning (from which measurements were made)

def thruster_outlines():
    """ 
    Returns 3 numpy arrays containing the outline of the thrusters.
    1st list item: Starboard azimuth thruster
    2nd list item: Port azimuth thruster
    3rd list item: Bow thruster
    
    within the np array:
    1st row: x-coordinate
    2nd row: y-coordinate

    These outlines are purely visual and do not represent the actual thruster geometry.
    """
    
    return [
        1.8*np.array([[0.006,-0.013,-0.013,-0.037,-0.037,-0.013,-0.013,0.006,0.006],[-0.0075,-0.0075,-0.023,-0.020,0.020,0.023,0.0075,0.0075,-0.0075 ]]),\
        1.8*np.array([[0.006,-0.013,-0.013,-0.037,-0.037,-0.013,-0.013,0.006,0.006],[-0.0075,-0.0075,-0.023,-0.020,0.020,0.023,0.0075,0.0075,-0.0075 ]]),\
        np.array([[0.10,-0.10,-0.075,0.075,0.10],[0.025,0.025,-0.025,-0.025,0.025]]) 
        ]

""" Manual control related """

def joy2act_TN_1(ax1,ax2,ax3,ax4,ax5):
    """Generates actuator output based on joystick input.
        Input: 4 joystick axes
        Output: 5 actuator values

        Intended for regular manual control of the Tito Neris vessel.

        Developed for Logitech Extreme 3D Pro
        Formulated by Bart Boogmans - 2022
    """

    # If the flick switch is up, use that for speed output
    axes_Spd = (ax4 +1)/2 # Normalized axes value between 0 and 1
    spd = 0.0
    if axes_Spd >= 0.05:
        spd = axes_Spd # Use flick switch for propeller speed
    else:
        spd = ax2 # Use forward tilt of main arm for propeller speed

    spd_aft_max = 3200 # Desired maximum speed of aft thrusters at full throttle (RPM
    rotation_aft_max = (1/3)*np.pi # Maximum rotation of aft thrusters (rad)
    bow_pwm_max = 0.3 # Maximum PWM value of bow thruster

    # returns array with actuators in order: SB_azimuth_propeller, PB_azimuth_propeller, bow_propeller, SB_azimuth_angle, PB_azimuth_angle
    return  [ 
            spd_aft_max*spd,
            spd_aft_max*spd,
            ax5*bow_pwm_max,
            -rotation_aft_max*ax1,
            -rotation_aft_max*ax1,
            ]