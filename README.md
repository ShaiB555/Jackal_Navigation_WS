# Jackal_Navigation_WS
A catkin_ws with the full requriements for running our final project, for estimation, control and navigation of a Jackal robot in an environement with obstacles using a Lidar sensor and known beacons. The README file explains how to use it.

#Must use ROS Noetic on Ubuntu 2020, then input these commands to the terminal to activate the simulation
cd Jackal_Navigation_WS
conda deactivate
catkin_make
source devel/setup.bash
roslaunch shaigalit_sim jackal_nav.launch x_d:="[[0.0],[0.0],[0.0]]"
#The above input x_d will define the desired position of the Jackal compared to the initial position
