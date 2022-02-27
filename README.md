# mr_janktastic

Collection of files needed to control the Mr. Janktastic robot arm.

### mrjank_control

This package contains the low-level interface to the Dynamixel AX-12A servos. Includes support for joint position, torque feedback control and applying effort to joints.

### mrjank_moveit_config

This package contains the MoveIt config files needed for planning and IK. For planning, RRTConnect is used. For kinematics, the KDL library is used. 

### mrjank_description

This package contains the URDF/Gazebo files needed for defining the robot's kinematic and dynamic configuration.
