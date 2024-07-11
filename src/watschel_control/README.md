# watschel_control

This package contains everything needed for controlling the watschel robot using ROS2 Control.

## Launch Files
- control.launch.py: Launch the controller manager, the JointTrajectoryController and the JointStateBroadcaster to control watschel using JointTrajectory messages.
- sim_control.launch.py: Only launch the Controllers and not the controller manager to control watschel within Gazebo Sim.