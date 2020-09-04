#!/usr/bin/env python

# Descrition: 
# at init load and publish global path 
# 
# in loop publish
# state (cartesian pose)
# broadcast tf map -> base_link

# ros
import rospy
import rospkg
import tf

# msgs
from ublox_sync.msg import GNSSFix
from xt28_localization.msg import XT28State
from xt28_localization.msg import Path
from nav_msgs.msg import Path as VisPath
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32

# various libs
import numpy as np
from lxml import etree 
import re
import utm
import yaml

class LocalizationNode:
    # constructor
    def __init__(self):
        # init node
        rospy.init_node('xt28_localization', anonymous=True)
        self.dt = rospy.get_param('/dt_state')
        self.dt_gp = rospy.get_param('/dt_pathglobal')
        self.rate = rospy.Rate(1./self.dt)      

        # timing
        self.pathglobal_pub_time_ratio = int(self.dt_gp/self.dt)

        # pubs and subs etc
        self.tfbr = tf.TransformBroadcaster()
        
        self.pathglobalpub = rospy.Publisher('global_plan', Path, queue_size=1)
        self.pathglobalpub_vis = rospy.Publisher('global_plan_vis', VisPath, queue_size=1)
        
        self.statepub = rospy.Publisher("state", XT28State, queue_size=1)
        self.state = XT28State()
        self.statetextmarkerpub = rospy.Publisher('state_text_marker', Marker, queue_size=1)
        
        self.gnsssub = rospy.Subscriber("/fix", GNSSFix, self.gnss_callback)
        self.fix = GNSSFix()
        self.received_gnss = False
        
        self.vx_sub = rospy.Subscriber("/speed_control/state", Float32, self.vx_callback)
        self.vx = 0.0
        self.received_vx = False
        
        self.beta0_sub = rospy.Subscriber("/steering_control/state", Float32, self.beta0_callback)
        self.beta0 = 0.0
        self.received_beta0 = False
        
        # load global plan 
        filename = rospy.get_param('/global_plan_filename')
        filepath = rospkg.RosPack().get_path('xt28_localization') + '/data/global_plans/' + filename
        if(filename.endswith('.yaml')):
            rospy.logwarn("xt28_loc: loading global plan from yaml")
            with open(filepath, 'r') as f:
                path_yaml = yaml.load(f,Loader=yaml.SafeLoader)
                lat = np.array(path_yaml["lat"])
                lon = np.array(path_yaml["lon"])
        elif(filename.endswith('.kml')):
            rospy.logwarn("xt28_loc: loading global plan from kml")
            tree = etree.parse(filepath)
            elements = tree.findall('.//{http://www.opengis.net/kml/2.2}Placemark') # each element is a "feature" in google earth
            ele_name = elements[0].find('.//{http://www.opengis.net/kml/2.2}name')
            if (ele_name.text == "global_plan" and \
                elements[0].find('.//{http://www.opengis.net/kml/2.2}LineString') is not None and \
                len(elements) == 1):  # check kml format
                    lat, lon, alt = self.coords_text_to_np(elements[0].find('.//{http://www.opengis.net/kml/2.2}coordinates').text)
            else:
                rospy.logerr("xt28_loc: Error loading kml global plan, kml file has wrong format, check README.md for correct format") 
            

        else: 
            rospy.logerr("xt28_loc: Error intializing, faulty global_plan_filename (.yaml or .kml only): " + filename)
            
        # convert global plan to utm
        Xgp_utm,Ygp_utm,utm_nr,utm_letter = utm.from_latlon(lat, lon)
    
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
                
                #### TMP!!!! CHECK WHY KOMATSU LOG IS 90 DEG OFF
                psi_raw = self.fix.heading-np.pi/2 
                psi_raw = self.angleToInterval(np.array([psi_raw]))[0]
                
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
            
                # publish text marker
                text =       "s:     " + "%.3f" % s + "\n"  \
                             "d:     " + "%.3f" % d + "\n"  \
                             "vx:    " + "%.3f" % self.vx + "\n" \
                             "beta0: " + "%.3f" % self.beta0
                m = self.get_text_marker(text)
                m.header.stamp = rospy.Time.now()
                self.statetextmarkerpub.publish(m) 
            
                # publish state (using only a subset of fields for now)
                self.state.header.stamp = rospy.Time.now()
                self.state.X = X_raw
                self.state.Y = Y_raw
                self.state.psi = psi_raw
                self.state.s = s
                self.state.d = d
                self.state.vx = self.vx
                self.state.beta0 = self.beta0
                self.statepub.publish(self.state)
                
            
            # publish global path (at lower rate) 
            if(count >= self.pathglobal_pub_time_ratio):
                pathglobal = Path()
                pathglobal.X = X_map
                pathglobal.Y = Y_map
                pathglobal.psi = psic
                pathglobal.s = s_map
                self.pathglobalpub.publish(pathglobal)
                
                # publish visualization msg as well
                pathglobalvis = VisPath()
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
                self.pathglobalpub_vis.publish(pathglobalvis)

                count = 0
            else:
                count +=1
            
            self.rate.sleep()
                        

    def gnss_callback(self, msg):
        self.fix = msg
        self.received_gnss = True
        
    def vx_callback(self, msg):
        self.vx = msg.data
        self.received_vx = True
    
    def beta0_callback(self, msg):
        self.beta0 = msg.data
        self.received_beta0 = True
        
    def angleToInterval(self,psi):
        try:         
            for i in range(psi.size):
                while(psi[i] > np.pi):
                    psi[i] = psi[i] -2*np.pi
                while(psi[i] <= -np.pi):
                    psi[i] = psi[i] +2*np.pi
            return psi
        except ValueError:
            print("Error in angleToInterval, psi.size = %i", psi.size)
    
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

    def coords_text_to_np(self,coords):
        # clean and cast
        coords = re.split(',| ',coords)
        coords[0] = coords[0].translate(None, "\n\t\t\t\t")
        coords = coords[:-1]
    
        # split in lon lat and alt
        if(len(coords) % 3 != 0):
            print "Error: len(coords) not divisible by three"
        lat = []
        lon = []
        alt = []
        for i in range(len(coords)/3):
            lon.append(float(coords[3*i]))
            lat.append(float(coords[3*i+1]))
            alt.append(float(coords[3*i+2]))
        lat = np.array(lat)
        lon = np.array(lon)
        alt = np.array(alt)
    
        return lat, lon, alt
 
    def get_text_marker(self,text):
        m = Marker()
        m.header.frame_id = "base_link"
        m.pose.position.x = 0.0;
        m.pose.position.y = 0.0;
        m.pose.position.z = 20.0;
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