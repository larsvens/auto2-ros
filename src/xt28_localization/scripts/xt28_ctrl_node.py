#!/usr/bin/env python

# Descrition: See doc/README.md

import numpy as np
import rospy
from xt28_localization.msg import XT28State
from xt28_localization.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from std_msgs.msg import Int16

class CtrlInterface:
    def __init__(self):
        
        # init node 
        rospy.init_node('xt_28_ctrl_interface', anonymous=True)
        self.dt = rospy.get_param('/dt_cmd')
        self.rate = rospy.Rate(1/self.dt)
        
        # subs
        self.pathglobalsub = rospy.Subscriber("global_plan", Path, self.pathglobal_callback)
        self.pathglobal = Path()
        self.pathglobal_received = False
        
        self.pathlocalsub = rospy.Subscriber("local_plan", Path, self.pathlocal_callback)
        self.pathlocal = Path()
        self.pathlocal_received = False
        
        self.state_sub = rospy.Subscriber("state", XT28State, self.state_callback)
        self.state = XT28State()
        self.state_received = False
        
        self.ctrlmodesub = rospy.Subscriber("ctrl_mode", Int16, self.ctrl_mode_callback)
        self.ctrl_mode = 0 # init to 0 (no cmds published)
        self.ctrl_mode_received = False
        
        self.gasrefsub = rospy.Subscriber("gasref", Float32, self.gasref_callback)
        self.gasref = 0.0 
        self.gasref_received = False
        
        # pubs
        self.lhptpub = rospy.Publisher('/lhpt_vis', Marker, queue_size=1)
        self.arcpub = rospy.Publisher('/arc_vis', Marker, queue_size=1)

        self.steering_cmd_pub = rospy.Publisher('/steering_command', Float32, queue_size=1)
        self.steering_cmd = Float32()

        self.throttle_cmd_pub = rospy.Publisher('/Gas', Float32, queue_size=1)
        self.throttle_cmd = Float32()

        self.ctrltextmarkerpub = rospy.Publisher('ctrl_text_marker', Marker, queue_size=1)

        # params (TODO ADJUST)
        self.L0 = 1.5 # from front axle to front articulated joint
        self.L1 = 1.5 # from front articulated joint to mid axle              
        
        # wait for messages before entering main loop
        while(not self.state_received):
            rospy.loginfo_throttle(1, "waiting for state")
            self.rate.sleep()

#        while(not self.ctrl_mode_received):
#            rospy.loginfo_throttle(1, "waiting for ctrl_mode")
#            self.rate.sleep
            
        # main loop
        while not rospy.is_shutdown(): 
            # wait for path
            if(self.ctrl_mode in [0,1,2]):
                while(not self.pathglobal_received):
                    rospy.loginfo_throttle(1, "waiting for pathglobal")
                    self.rate.sleep()
            elif(self.ctrl_mode == 3):
                while(not self.pathlocal_received):
                    rospy.loginfo_throttle(1, "waiting for pathlocal")
                    self.rate.sleep()
            # compute and visualize ctrl
            rospy.loginfo_throttle(1,"XT28_ctrl_interface: TRACKING, ctrl_mode = " + str(self.ctrl_mode)) 
            lhdist = 10
            if(np.abs(self.state.d) > lhdist):
                rospy.logwarn_throttle(1,"XT28_ctrl_interface: WARNING: dist to path larger than lhdist" )
            rho = self.pp_curvature(self.state, self.pathglobal, lhdist)                
            beta_ref = rho*(self.L0+self.L1)
            

            # publish ctrl text marker
            text =      "ctrl_mode:    " + "%.3f" % self.ctrl_mode + "\n"  \
                        "steering_cmd: " + "%.3f" % beta_ref + "\n"  \
                        "throttle_cmd: " + "%.3f" % self.gasref 
            if(self.ctrl_mode in [0,1]): 
                m = self.get_text_marker(text,1,0,0) # red means not published
            else:
                m = self.get_text_marker(text,0,1,0) # green means published
            m.header.stamp = rospy.Time.now()
            self.ctrltextmarkerpub.publish(m)                            

            # publish ctrl 
            if(self.ctrl_mode == 0):
                rospy.loginfo_throttle(1,"XT28_ctrl_interface: STOP, ctrl_mode = " + str(self.ctrl_mode) + " (no cmd published)") 
            elif(self.ctrl_mode == 1):
                rospy.loginfo_throttle(1,"XT28_ctrl_interface: STOP, ctrl_mode = " + str(self.ctrl_mode) + " (publishing zero cmds)") 
                self.publish_cmds(0.0,0.0)
            elif(self.ctrl_mode in [2,3]):               
                self.publish_cmds(beta_ref,self.gasref)
            else:
                rospy.logerr("XT28_ctrl_interface: invalid ctrl_mode! ctrl_mode = " + str(self.ctrl_mode))
            self.rate.sleep()
  
    def publish_cmds(self,beta_ref,gas_ref):
        # publish steering cmd
        self.steering_cmd.data = beta_ref
        self.steering_cmd_pub.publish(self.steering_cmd)
        # publish throttle cmd
        self.throttle_cmd.data = gas_ref
        self.throttle_cmd_pub.publish(self.throttle_cmd)
    
    def pp_curvature(self, state, path, lhdist):
        
        s_lh = state.s + lhdist
        Xlh = np.interp(s_lh, path.s, path.X)
        Ylh = np.interp(s_lh, path.s, path.Y) 
        
        deltaX = (Xlh-state.X)
        deltaY = (Ylh-state.Y)
        lh_dist = np.sqrt(deltaX**2 + deltaY**2)
        lh_angle = np.arctan2(deltaY,deltaX) - state.psi
        #lh_angle = angleToInterval(np.array([lh_angle]))[0]
        rho_pp = 2*np.sin(lh_angle)/lh_dist    
        
        # return curve for visualization
        if(rho_pp != 0.):
            R = np.clip(1./rho_pp, a_min = -10000, a_max = 10000)
        else:
            R = 10000
        Nvis = 10
        if(np.abs(lh_angle) > 0.005):
            t = np.linspace(0.,2*lh_angle,Nvis)
            xarc = R*np.sin(t) # - self.lr
            yarc = R-R*np.cos(t)
        else:
            xarc = np.linspace(0.,lh_dist,Nvis) # - self.lr
            yarc = np.zeros(Nvis)        

        # publish vis marker
        m_arc = self.get_arc_marker(xarc, yarc)
        self.arcpub.publish(m_arc)
        m = self.get_lhpt_marker(Xlh,Ylh)
        self.lhptpub.publish(m) 

        return rho_pp


    def get_arc_marker(self, x, y):
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "base_link"        
        # assume pose initialized to zero
        m.type = m.LINE_STRIP
        m.points = []
        for i in range(x.size):
            p = Point()
            p.x = x[i]
            p.y = y[i]
            p.z = 0.2
            m.points.append(p)
        m.scale.x = 0.2;
        m.color.a = 1.0; 
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        return m

    def get_lhpt_marker(self,Xlh,Ylh):
        m = Marker()
        m.header.stamp = rospy.Time.now()
        m.header.frame_id = "map"
        m.pose.position.x = Xlh;
        m.pose.position.y = Ylh;
        m.pose.position.z = 0.1;
        m.type = m.SPHERE;
        m.scale.x = 0.6;
        m.scale.y = 0.6;
        m.scale.z = 0.6;
        m.color.a = 1.0; 
        m.color.r = 0.0;
        m.color.g = 0.0;
        m.color.b = 1.0;
        return m

    def get_text_marker(self,text,r,g,b):
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
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        return m
    
    def pathglobal_callback(self, msg):
        self.pathglobal = msg
        self.pathglobal_received = True
  
    def pathlocal_callback(self, msg):
        self.pathlocal = msg
        self.pathlocal_received = True

    def state_callback(self, msg):
        self.state = msg 
        self.state_received = True
        
    def ctrl_mode_callback(self, msg):
        self.ctrl_mode = msg.data
        self.ctrl_mode_received = True
        
    def gasref_callback(self, msg):
        self.gasref = msg.data
        self.gasref_received = True
    

if __name__ == '__main__':
    ci = CtrlInterface()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
