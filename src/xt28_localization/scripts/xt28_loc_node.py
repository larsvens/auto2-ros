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
from visualization_msgs.msg import Marker
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
        self.dt_gp = rospy.get_param('/dt_pathglobal')
        self.rate = rospy.Rate(1./self.dt)      

        # timing
        self.pathglobal_pub_time_ratio = int(self.dt_gp/self.dt)

        # pubs and subs etc
        self.tfbr = tf.TransformBroadcaster()
        self.pathglobalpub = rospy.Publisher('pathglobal', Path, queue_size=1)
        self.statetextmarkerpub = rospy.Publisher('state_text_marker', Marker, queue_size=1)
        self.gnsssub = rospy.Subscriber("/fix", GNSSFix, self.gnss_callback)
        self.fix = GNSSFix()
        self.received_gnss = False
        
        # load global plan yaml
        filename = rospy.get_param('/global_plan_filename')
        filepath = rospkg.RosPack().get_path('xt28_localization') + '/data/global_plans_yaml/' + filename
        with open(filepath, 'r') as f:
            path = yaml.load(f,Loader=yaml.SafeLoader)

        # convert to utm
        Xgp_utm,Ygp_utm,utm_nr,utm_letter = utm.from_latlon(np.array(path["lat"]), np.array(path["lon"]))
    
        # set path in map coord sys (filter to ca 1pt/m)
        X_map_tmp = Xgp_utm - Xgp_utm[0]
        Y_map_tmp = Ygp_utm - Ygp_utm[0]
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
        s_map = np.arange(0,s_tmp[-1],ds)
        X_map = np.interp(s_map,s_tmp,X_map_tmp)
        Y_map = np.interp(s_map,s_tmp,Y_map_tmp)
        
        # compute psic
        dX = np.diff(X_map)
        dY = np.diff(Y_map)
        psic = np.arctan2(dY,dX)
        #psic_final = np.arctan2(Y_map[0]-Y_map[-1],X_map[0]-X_map[-1])  # assuming closed path
        psic_final = psic[-1] # assuming open path
        psic = np.append(psic,psic_final)         
     
        # set utm origin pose from first entry and tangent in global plan 
        X0_utm = float(Xgp_utm[0])
        Y0_utm = float(Ygp_utm[0])
        psi0_utm = float(np.arctan2(Y_map[1]-Y_map[0],X_map[1]-X_map[0])) # map and utm has same orientation and since X/Y_map has 1m distance this is more robust
                
        self.origin_pose_utm =	{
          "X0_utm": X0_utm,
          "Y0_utm": Y0_utm,
          "psi0_utm": psi0_utm,
          "utm_nr": utm_nr, 
          "utm_letter": utm_letter
        }
        
        # check
        print X_map.size

        # Main loop
        count  = 0
        while not rospy.is_shutdown():
            
            # get transform map-->base_link
            if(not self.received_gnss):
                rospy.logwarn_throttle(1.0,"xt28_loc: waiting for gnss message")
            else:
                X_utm, Y_utm, utm_nr, utm_letter = utm.from_latlon(self.fix.latitude, self.fix.longitude)
                X_raw = X_utm - self.origin_pose_utm["X0_utm"]
                Y_raw = Y_utm - self.origin_pose_utm["Y0_utm"]
                psi_raw = self.fix.heading
                
                # check utm zone
                if(utm_nr != self.origin_pose_utm["utm_nr"] or utm_letter != self.origin_pose_utm["utm_letter"]):
                    rospy.logerr("UTM zone mismatch: GPS measurement utm_nr =     " + str(utm_nr) + ", origin_pose utm_nr =     " + str(self.origin_pose_utm["utm_nr"]))
                    rospy.logerr("UTM zone mismatch: GPS measurement utm_letter = " + utm_letter + ", origin_pose utm_letter = " + str(chr(self.origin_pose_utm["utm_letter"])))                    

                # publish transform
                self.tfbr.sendTransform((X_raw, Y_raw, 0),
                        tf.transformations.quaternion_from_euler(0, 0, psi_raw),
                        rospy.Time.now(),
                        "base_link",
                        "map")
            
                # get s and d position relative to pathglobal
                s, d = self.ptsCartesianToFrenet(np.array([X_raw]),np.array([Y_raw]),X_map,Y_map,psic,s_map)
                rospy.logwarn("xt28_loc: s = " + str(s))
                rospy.logwarn("xt28_loc: d = " + str(d))
            
                # publish text marker
                state_text = "s:     " + "%.3f" % s + "\n"  \
                             "d:     " + "%.3f" % d 
                m = self.get_state_text_marker(state_text)
                m.header.stamp = rospy.Time.now()
                self.statetextmarkerpub.publish(m) 
            
            # publish global path (at lower rate) 
            if(count >= self.pathglobal_pub_time_ratio):
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
                self.pathglobalpub.publish(pathglobalvis)

                count = 0
            else:
                count +=1
            
            
            #rospy.logwarn("xt28_loc: in main loop")
            self.rate.sleep()
                        

    def gnss_callback(self, msg):
        self.fix = msg
        self.received_gnss = True
        

    
    def ptsCartesianToFrenet(self,X,Y,Xpath,Ypath,psipath,spath):
        # inputs and outputs are np.array([x])
    
        Npts = X.size
        s = np.zeros(Npts);
        d = np.zeros(Npts);
        
        # transform pts one at a time
        for k in range(Npts):
        
            # find closest pt on centerline
            dist = np.sqrt((X[k]-Xpath)**2 + (Y[k]-Ypath)**2 )
            idx = np.argmin(dist)
    
            # make sure deltas is positive
            deltas = -1
            maxiters = 5
            iters = 0
            while(deltas < 0 and iters <= maxiters):
            
                Xc = Xpath[idx] 
                Yc = Ypath[idx] 
                psic = psipath[idx] 
                sc = spath[idx] 
                
                # compute angles etc
                deltaX = X - Xc
                deltaY = Y - Yc
                alpha1 = -np.arctan2(deltaX,deltaY)
                alpha2 = psic - alpha1
                M = np.sqrt(deltaX**2 + deltaY**2)
                deltas = M*np.sin(alpha2)
                
                # iteratively reduce idx until deltas positive
                idx = idx - 1
                iters = iters + 1
                
            d[k] = M*np.cos(alpha2)
            s[k] = sc + deltas
            
        return s,d 
 
    def get_state_text_marker(self,text):
        m = Marker()
        m.header.frame_id = "base_link"
        m.pose.position.x = 0.0;
        m.pose.position.y = 0.0;
        m.pose.position.z = 10.0;
        m.type = m.TEXT_VIEW_FACING;
        m.text = text
        m.scale.x = 2.0;
        m.scale.y = 2.0;
        m.scale.z = 2.0;
        m.color.a = 1.0; 
        m.color.r = 0.0;
        m.color.g = 0.0;
        m.color.b = 0.0;
        return m
           

if __name__ == '__main__':
    ln = LocalizationNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")