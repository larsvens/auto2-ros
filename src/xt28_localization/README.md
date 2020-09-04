

# Usage Instructions

## Get a Global Plan    
This can be done by either   
1. Manually drive/walk/log a gnss trace stored as .bag 
2. Draw a path in google Earth and export .kml   

In case of 1: Use script `global_plan_to_yaml.py` to generate a `.yaml` file in `data/global_plans_yaml`     
In case of 2 (which is the easier method btw): 
1. open google earth in your browser, 
2. draw a path using the "draw a line or shape"-tool, (do not draw a closed shape, then it becomes a different data type in the .kml) 
3. press enter to save the line, 
4. enter "global_plan" in the place title field
5. expand "add to project" and select "new project"
6. put a reasonable filename in "new project title" e.g., jalla_test_track
7. click "more actions" within your new project and select "export as kml file"
8. place the .kml file in /data/global_plans

When you have generated a global plan file (.kml or .yaml), set the parameter `/global_plan_filename` in `xt28_loc.launch` to the filename (with extension)  


## The Localization Node    

The node subscribes to    
* `/fix` of type ublox_sync/GNSSFix defined in sensor_interface package   

And publishes   
* `/global_plan` of type `xt28_localization/Path` at a fixed lower rate set by param `/dt_pathglobal`   
* `/global_plan_vis` of type `nav_msgs/Path` for rviz   
* `/state` of type `xt28_localization/XT28State` at a fixed higher rate set by param `dt_state`   

It also broadcasts TF transform map -> base_link   

To start the node: `roslaunch xt28_localization xt28_loc.launch`    



## The Control Node
At the moment, the package also contains the xt28_ctrl_interface node for convenience.   

The node subscribes to 
* `/ctrl_mode` of type `std_msgs/Int16`      
* `/state` of type `xt28_localization/XT28State`   
* `/global_plan` of type `xt28_localization/Path`   
* `/local_plan` of type `xt28_localization/Path`   

And publishes   
* `/steering_command` of type `std_msgs/Float32`   
* `/Gas` of type `std_msgs/Float32`   
at a fixed rate set by param `/dt_cmd`    

Node behavior set by ctrl_mode topic   
* 0: stop (no cmds published)   
* 1: stop (zeros published)   
* 2: track_global_plan   
* 3: track_local_plan   
In modes 0 and 1 control commands are computed (but not published) and visualized (red text) in rviz   
In modes 2 and 3 control commands are computed, published and visualized (green text) in rviz   

The control command and default reference throttle can be input from the `xt28_live_input` rqt window that is included in the launch file.    

To start the node: `roslaunch xt28_localization xt28_ctrl.launch`    



# Setup Instructions
The repository is a self contained ros package, so the install procedure is just to clone the repository into    
`whatever_catkin_ws/src` and `cd .. && catkin_make`    

In addition to your standard ROS install the following python packages are required (may need to specify versions, and add more, let's see):    
* `numpy`   
* `lxml`   
* `utm`   

# Test Instructions
To try out the node and see if your setup is ok run the following in three terminals (make sure all terminals have sourced the workspace)
* `roslaunch xt28_localization xt28_loc.launch`
* `roslaunch xt28_localization xt28_ctrl.launch`
* `rosbag play 200515_tomtebo_walk_backpack_gnssfix_cropped.bag` (this needs to be done from /data/bags)






