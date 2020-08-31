# Intended Usage

## Get a global plan    
This can be done by either   
* Draw a path in google Earth and export .kml   
* Manually drive/walk/log a gnss trace stored as .bag   
Use script global_plan_to_yaml.py to convert to easily loadable format (TODO kml conversion)   
The script will generate a .yaml file in data/global_plans_yaml   

## Run the localization node    
Set parameters   
* `/global_plan_filename` set to the filename of the generated .yaml file   

The node subscribes to    
* `/fix` of type ublox_sync/GNSSFix defined in sensor_interface package   

And publishes   
* `/global_plan` of type nav_msgs/Path   
* `/utm_origin` of type ??   
* `/state` of type ??

It also broadcasts TF transform map -> base_link


