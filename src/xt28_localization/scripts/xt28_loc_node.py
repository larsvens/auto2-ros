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
from nav_msgs.msg import Path as Path
from geometry_msgs.msg import PoseStamped
import tf

# custom msgs
from ublox_sync.msg import GNSSFix

# compute tools
import numpy as np

# coordinate system libs
import utm

# misc
import yaml



class LocalizationNode:
    # constructor
    def __init__(self):
        # init node
        rospy.init_node('xt28_localization', anonymous=True)
        self.dt = rospy.get_param('/dt_localization')
        self.rate = rospy.Rate(1./self.dt)      

        # pubs and subs
        self.pathglobalpub = rospy.Publisher('pathglobal_vis', Path, queue_size=1)
        self.gnsssub = rospy.Subscriber("/fix", GNSSFix, self.gnss_callback)
        self.fix = GNSSFix()
        self.received_gnss = False
        
        # load global plan yaml
        filename = rospy.get_param('/global_plan_filename')
        filepath = rospkg.RosPack().get_path('xt28_localization') + '/data/global_plans_yaml/' + filename
        with open(filepath, 'r') as f:
            path = yaml.load(f,Loader=yaml.SafeLoader)

        # convert to utm
        X_utm,Y_utm,utm_nr,utm_letter = utm.from_latlon(np.array(path["lat"]), np.array(path["lon"]))
    
        # set path in map coord sys (filter to ca 1pt/m)
        X_map_tmp = X_utm - X_utm[0]
        Y_map_tmp = Y_utm - Y_utm[0]
        #X_map_tmp = np.append(X_map_tmp, X_map_tmp[0]) # for closed path only
        #Y_map_tmp = np.append(Y_map_tmp, Y_map_tmp[0])
    
        # compute s 
        dX = np.diff(X_map_tmp)
        dY = np.diff(Y_map_tmp)
        ds = np.sqrt(dX**2+dY**2)
        s_tmp = np.cumsum(ds)
        s_tmp = np.append(0, s_tmp)
        
        # resample with equidistant points
        ds = 1.0
        s = np.arange(0,s_tmp[-1],ds)
        X_map = np.interp(s,s_tmp,X_map_tmp)
        Y_map = np.interp(s,s_tmp,Y_map_tmp)
        
        # compute psic
        dX = np.diff(X_map)
        dY = np.diff(Y_map)
        psic = np.arctan2(dY,dX)
        #psic_final = np.arctan2(Y_map[0]-Y_map[-1],X_map[0]-X_map[-1])  # assuming closed path
        psic_final = psic[-1] # assuming open path
        psic = np.append(psic,psic_final)         
     
        # set utm origin pose from first entry and tangent in global plan 
        X0_utm = float(X_utm[0])
        Y0_utm = float(Y_utm[0])
        psi0_utm = float(np.arctan2(Y_map[1]-Y_map[0],X_map[1]-X_map[0])) # map and utm has same orientation and since X/Y_map has 1m distance this is more robust
                
        origin_pose_utm =	{
          "X0_utm": X0_utm,
          "Y0_utm": Y0_utm,
          "psi0_utm": psi0_utm,
          "utm_nr": utm_nr, 
          "utm_letter": utm_letter
        }
        
        # publish global plan as path
        pathglobalvis = Path()
        for i in range(X_map.size):
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "map"
            pose.pose.position.x = X_map[i]
            pose.pose.position.y = Y_map[i]            
            quaternion = tf.transformations.quaternion_from_euler(0, 0, psic[i])
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            pathglobalvis.poses.append(pose)
        pathglobalvis.header.stamp = rospy.Time.now()
        pathglobalvis.header.frame_id = "map"
        #rospy.logwarn("track_iface: publishing pathglobal visualization")
        self.pathglobalpub.publish(pathglobalvis)


        # Main loop
        while not rospy.is_shutdown():
            #rospy.logwarn("xt28_loc: in main loop")
            self.rate.sleep()
                        

    def gnss_callback(self, msg):
        self.fix = msg
        self.received_gnss = True
        rospy.logwarn("xt28_loc: in callback")
            

if __name__ == '__main__':
    ln = LocalizationNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")