#!/usr/bin/env python

# Descrition: 
# at init load and publish 
# * global path 
# * utm origin
# 
# in loop publish
# state (cartesian pose)
# broadcast tf

# ros
import rospy
import rospkg

# compute tools
import numpy as np
from scipy import interpolate

# coordinate system libs
import utm

# various dataformats
#import simplekml
from lxml import etree 
import re
import yaml



class LocalizationNode:
    # constructor
    def __init__(self):
        # init node
        rospy.init_node('xt28_localization', anonymous=True)
        self.dt = rospy.get_param('/dt_localization')
        self.rate = rospy.Rate(1./self.dt)      



        # load global plan yaml
        filename = rospy.get_param('/global_plan_filename')
        filepath = rospkg.RosPack().get_path('xt28_localization') + '/data/global_plans_yaml/' + filename
        with open(filepath, 'r') as f:
            path = yaml.load(f,Loader=yaml.SafeLoader)

        # convert to utm
        X_utm,Y_utm,utm_nr,utm_letter = utm.from_latlon(np.array(path["lat"]), np.array(path["lon"]))

        # filter to approx 1 pt / m ? 
        
        
        # get utm origin from first entry and tangent in global plan 


        # publish


        # Main loop
        while not rospy.is_shutdown():
            rospy.logwarn("xt28_loc: in main loop")
            self.rate.sleep()
            
            
            


            
            
            

if __name__ == '__main__':
    ln = LocalizationNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")