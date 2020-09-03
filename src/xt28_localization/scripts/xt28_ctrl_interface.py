#!/usr/bin/env python

# Temporarily placed in xt28_localization for convenience

# Descrition: Produces control signals to track a path (global or local)

# subscribes:
# state from stateestimation node (topic /state)
# local or global path from to track (topic /pathlocal)

# publishes: 
# vehicle specific ctrl command 

import numpy as np
import rospy
from auto2_common.msg import XT28State
from auto2_common.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from std_msgs.msg import Int16

class CtrlInterface:
    def __init__(self):
        
        # init node 
        rospy.init_node('xt_28_ctrl_interface', anonymous=True)
        self.dt = 0.05
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
        self.ctrl_mode = 1 # 0: stop, 1: track_global_plan, 2: track_local_plan
        self.ctrl_mode_received = False
        
        self.vxrefsub = rospy.Subscriber("vxref", Float32, self.vxref_callback)
        self.vxref = 1.0 
        self.vx_ref_received = False
        
        # pubs
        self.lhptpub = rospy.Publisher('/lhpt_vis', Marker, queue_size=1)
        self.arcpub = rospy.Publisher('/arc_vis', Marker, queue_size=1)
        #self.cmdpub = rospy.Publisher('OpenDLV/ActuationRequest', ActuationRequest, queue_size=1)

        # params (TODO ADJUST)
        self.L0 = 1.5 # from front axle to front articulated joint
        self.L1 = 1.5 # from front articulated joint to mid axle              
        
        # wait for messages before entering main loop
        while(not self.state_received):
            rospy.loginfo_throttle(1, "waiting for state")
            self.rate.sleep()

        if(self.ctrl_mode == 1):
            while(not self.pathglobal_received):
                rospy.loginfo_throttle(1, "waiting for pathglobal")
                self.rate.sleep()
        elif(self.ctrl_mode == 2):
            while(not self.pathlocal_received):
                rospy.loginfo_throttle(1, "waiting for pathlocal")
                self.rate.sleep()

#        while(not self.ctrl_mode_received):
#            rospy.loginfo_throttle(1, "waiting for ctrl_mode")
#            self.rate.sleep
            
        # main loop
        while not rospy.is_shutdown(): 
            # COMPUTE CONTROL
            if(self.ctrl_mode == 0):
                rospy.loginfo_throttle(1,"XT28_ctrl_interface: STOP, ctrl_mode = " + str(self.ctrl_mode)) 
                # no cmd published
            elif(self.ctrl_mode in [1,2]):
                rospy.loginfo_throttle(1,"XT28_ctrl_interface: TRACKING, ctrl_mode = " + str(self.ctrl_mode)) 
                lhdist = 10
                rho = self.pp_curvature(self.state, self.pathglobal, lhdist)                
                beta_ref = rho*(self.L0+self.L1)
                rospy.loginfo_throttle(1,"XT28_ctrl_interface: beta_ref = " + str(beta_ref)) 
                vx_ref = 0
                # publish cmd
                #self.cmdpub.publish(self.cmd)                                 
            else:
                rospy.logerr("XT28_ctrl_interface: invalid ctrl_mode! ctrl_mode = " + str(self.ctrl_mode))

            self.rate.sleep()
            
    
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
        
    def vxref_callback(self, msg):
        self.cc_vxref = msg.data
        self.vx_ref_received = True
    

if __name__ == '__main__':
    ci = CtrlInterface()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
