USEFUL TERMINAL COMMANDS:
cd catkin_ws
conda deactivate
source devel/setup.bash
catkin_make
roslaunch shaigalit_sim jackal_nav.launch x_d:="[[0.0],[0.0],[0.0]]"



BELOW NOT NEEDED!!!!
roslaunch shaigalit_sim world_call.launch config:=front_laser
roslaunch jackal_viz view_robot.launch
rqt_graph
cd ~/catkin_ws/src/shaigalit_sim/scripts
rosrun shaigalit_sim laser_sub.py

THINGS TO DO NEXT TIME:
Fix error in x position of beacon??? 0.1m error is a lot!
Finish the EKF: 
**EKF does not converge into real robot position
**after a while there is a singularity due to division by zero in H_calc and theta calculation (somewhere) iside the arctan of theta, meaning that xb=xr which makes no sense unless the robot is in the same place as the beacon (it isn't)
find the Jacobian of the measurments - H. Try using a function that calculates the Jacobian (symbolic???)

Beacons:
0.25 - (6,6)
0.31 - (-6,-6)
0.46 - (-6,6)
0.54 - (6,-6)
