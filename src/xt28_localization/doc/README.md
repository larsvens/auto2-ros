# Intended Usage

## Get a Global Plan    
This can be done by either   
* Draw a path in google Earth and export .kml   
* Manually drive/walk/log a gnss trace stored as .bag   
Use script global_plan_to_yaml.py to convert to easily loadable format (TODO kml conversion)   
The script will generate a .yaml file in data/global_plans_yaml   

## The Localization Node    
Set parameters   
* `/global_plan_filename` set to the filename of the generated .yaml file   

The node subscribes to    
* `/fix` of type ublox_sync/GNSSFix defined in sensor_interface package   


And publishes   
* `/global_plan` of type `xt28_localization/Path` at a fixed lower rate set by param `/dt_pathglobal`   
* `/global_plan_vis` of type `nav_msgs/Path` for rviz   
* `/state` of type `xt28_localization/XT28State` at a fixed higher rate set by param `dt_state`   

It also broadcasts TF transform map -> base_link   

# Setup Instructions
The repository is a self contained ros package, so the install procedure is just to clone the repository into    
`whatever_catkin_ws/src` and `cd .. && catkin_make`    

In addition to your standard ROS install the following python packages are required:    
* `numpy`   
* `lxml`   
* `utm`   

## The Control Interface Node
At the moment, the package also contains the xt28_ctrl_interface node for convenience.   

The node subscribes to    
* `/state` of type `xt28_localization/XT28State`   
* `/global_plan` of type `xt28_localization/Path`   
* `/local_plan` of type `xt28_localization/Path`   

And publishes   
* `/steering_command` of type `std_msgs/Float32`   
* `/Gas` of type `std_msgs/Float32`   
at a fixed rate set by param `/dt_cmd`    
