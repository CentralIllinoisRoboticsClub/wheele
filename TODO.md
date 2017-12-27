# Wheele To Do List:

## ROS nodes
Draw up some node diagrams (ros rqt_graph) that show how wheele will function.
(e.g. https://answers.ros.org/question/268606/need-help-with-basic-navigation/)

### Vehicle Model
Subscribes to speed and curvature (/wheele_cmd_vel).
Publishes left/right throttle, left/right steer (/thrush_and_steer).

### MicroMaestro
Subcribes to left/right throttle, left/right steer (/thrush_and_steer).
Sends 6 servo control serial commands over USB to Micro Maestro board.
Plan to update this for a micro that receives CAN msgs and sends serial to Micro Maestro
I propose we send the 6 raw servo/esc commands over CAN from this node. Any tuning/etc can be edited on the pi.

### can2ros
Receives can messages using the python-can library.
Publishes /wheele_cmd_vel for manual mode
Need to calculate and publish ros /odom using encoder and/or gyro data.

### leddar_to_scan
Should we consider a scanse sweep?

### get_waypoints, path_planning, obstacle_avoidance

### Do we need a map? probably good for at least leveraging local path planner packages

### cone_tracker with camera
