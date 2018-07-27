# Wheele ROS commands
python setup_gps.py
roslaunch ros_vehicle_model wheele_master.launch
roslaunch circ_2dnav circ_move_base_nav.launch
rosrun ros_vehicle_model wheele_local_planner.py
sudo /sbin/ip link set can0 up type can bitrate 500000

rosrun ros_vehicle_model conv_cmd_vel.py

sudo ip link set can0 down

# Notes, July 27, 2018
Goals:
1) Leddar filter (ground, noise)
- Calculated roll and pitch and broadcast base_link to laser transform
Testing this did not give desired results. Ground elev did not seem to be lower than leddar.
Next try setting leddar Z 0.3 m above base_link to get the appropriate lever arm effects.
I just noticed that wheele_master.launch still used the static tf laser->base_link. Hopefully that explains it...

2) Waypoints with cost map but no pre-loaded map
