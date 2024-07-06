# brunhilde_control

This package contains everything needed for controlling the Brunhilde robot using ROS2 Control.

## Launch Files
- control.launch.py: Launch the controller manager, the JointTrajectoryController and the JointStateBroadcaster to control Brunhilde using JointTrajectory messages.
- sim_control.launch.py: Only launch the Controllers and not the controller manager to control Brunhilde within Gazebo Sim.