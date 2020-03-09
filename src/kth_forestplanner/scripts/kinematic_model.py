#!/usr/bin/env python

# set ctrl strategy:
# 1 nominal: beta1dot = k*beta0dot
# 2 single feedforward term (compute beta0dot from diff(beta0,beta1))
# 3 track compute psidot2 corresponding to no actuation, set beta1dot accordingly

ctrl_strategy = 3
saturate_beta1prime = 0

import numpy as np
import matplotlib.pyplot as plt 
plt.rcParams['figure.dpi'] = 200 # default 100
plt.rcParams['figure.figsize'] = 10, 10

def set_input_sequence(beta0dot_seq,v0_seq,dt):
    # 1 second of ctrl vals for each value in u (list)    
    N_per_sec = int(1/dt)
    betadot_des_arr = []
    v0_arr = []
    for i in range(len(beta0dot_seq)):
        betadot_des_arr = betadot_des_arr + (beta0dot_seq[i]*np.ones(N_per_sec)).tolist()  
    t = np.arange(0,len(v0_seq),dt)
    v0_arr = np.interp(t, np.arange(len(v0_seq)), v0_seq)
    #plt.figure(1)
    #plt.plot(t,v0_arr) # to check v0 
    return np.array(betadot_des_arr), np.array(v0_arr)

def get_axle_position(X,Y,psi,w):
    Xl = X - (0.5*w)*np.sin(psi)
    Yl = Y + (0.5*w)*np.cos(psi)
    Xr = X + (0.5*w)*np.sin(psi)
    Yr = Y - (0.5*w)*np.cos(psi)
    return np.array([[Xl,Xr,Yl,Yr]])
  
dt = 0.001
# set high level ctrl sequence
betadot_des_seq = [0.5, 0.5, -0.8, -0.8, -0.8, 0.8, 0.0, -0.8, -0.5, -0.5, 0.5]
v0_seq =       [0.0, 0.0, 0.5, 0.75, 1.0, 1.0, 0.5, -0.5, -1.0, -1.0, -0.5]
#beta0dot_seq = [0.5, 0.5, -0.8, -0.8, -0.8]
#v0_seq = [-1.0, -1.0, -0.5, -0.5, -0.5]


betadot_des_arr, v0_arr = set_input_sequence(betadot_des_seq,v0_seq,dt)
N = int((1/dt))*len(betadot_des_seq)

# params (small scale)
L0 = 0.5 # from front axle to front articulated joint
L1 = 0.5 # from front articulated joint to mid axle
L2 = 0.3 # from mid axle to rear articulated joint
L3 = 0.3 # from rear articulated joint to rear axle
w = 0.5

# params (full scale)
#L0 = 1.5 # from front axle to front articulated joint
#L1 = 1.5 # from front articulated joint to mid axle
#L2 = 0.5 # from mid axle to rear articulated joint
#L3 = 1.5 # from rear articulated joint to rear axle
#w = 2.5

# set init state 
X0 = 0
Y0 = 0
psi0 = 0
beta0 = 0 
X1 = -(L0+L1) 
Y1 = 0
psi1 = 0
beta1 = 0
beta1dot = 0.0
X2 = -(L0+L1+L2+L3)
Y2 = 0
psi2 = 0

# init help vars
v1 = v0_arr[0]
v2 = v0_arr[0]
psidot0 = 0

# loop
t_arr = [0]
X0_arr = [X0]
Y0_arr = [Y0]
psi0_arr = [psi0]
beta0_arr = [beta0]
X1_arr = [X1]
Y1_arr = [Y1]
psi1_arr = [psi1]
beta1_arr = [beta1]
X2_arr = [X2]
Y2_arr = [Y2]
psi2_arr = [psi2]
beta1dot_arr = [beta1dot]
v1_arr = [v1]
v2_arr = [v2]
axle_pos_arr = []
reversing = False
plt.figure(2)
for i in range(N):
    # control          
    if (ctrl_strategy == 1): # nominal
        beta0dot = betadot_des_arr[i]
        v0 = v0_arr[i]
        beta1ref = 0.7*beta0dot
        beta1dot = 0.5*(beta1ref-beta1)
        
    if (ctrl_strategy == 2): # beta0-beta1 feedfwd
        v0 = v0_arr[i]
        if(-0.1 < v0 < 0.1): # standing stil
            beta1dot =-beta0dot*(L2/L3) # todo track angle instead
            #beta1dot = 0
        elif (v0 >= 0.1): # forward
            beta0dot = betadot_des_arr[i]
            beta1prime_des = (beta0-beta1)/(L1+L2)
            beta1dot = beta1prime_des*v0
        else: # reverse
            beta1dot = betadot_des_arr[i]    
            beta0prime_des = (beta1-beta0)/(L1+L2)
            beta0dot = beta0prime_des*v0
            
    if (ctrl_strategy == 3):
        v0 = v0_arr[i]
        if(v0 > -0.01): # fwd
            beta0dot = betadot_des_arr[i]
            psidot1 = (v1*np.sin(beta0)-L0*beta0dot)/(L0 + L1*np.cos(beta0))
            psidot2_des = (v1*np.sin(beta1) - psidot1*L2*np.cos(beta1))/L3
            beta1dot = psidot1 - psidot2_des
        else: # reverse
            beta1dot = betadot_des_arr[i]    
            psidot1 = (v1*np.sin(beta1)-L3*beta1dot)/(L3 + L2*np.cos(beta1))
            psidot0_des = (v1*np.sin(beta0) - psidot1*L1*np.cos(beta0))/L0
            beta0dot = psidot1 - psidot0_des         
           
    # dynamics (from altafini 1999 IJRR, extended to three bodies)
    if (v0 > -0.01): # fwd
        beta0 = beta0 + dt*beta0dot
        v1 = ((L0+L1*np.cos(beta0))*v0 + L0*L1*np.sin(beta0)*beta0dot)/(L1+L0*np.cos(beta0))
        psidot0 = (v1*np.sin(beta0) + L0*np.cos(beta0)*beta0dot)/(L0+L1*np.cos(beta0))
        psi0 = psi0 + dt*psidot0
        X0 = X0 + dt*v0*np.cos(psi0)
        Y0 = Y0 + dt*v0*np.sin(psi0)
        psi1 = psi0 - beta0
        X1 = X1 + dt*v1*np.cos(psi1)
        Y1 = Y1 + dt*v1*np.sin(psi1)
        beta1 = beta1 + dt*beta1dot
        psi2 = psi1-beta1
        Xj1 = X1 - L2*np.cos(psi1)
        Yj1 = Y1 - L2*np.sin(psi1)
        X2 =  Xj1 - L3*np.cos(psi2)
        Y2 = Yj1 - L3*np.sin(psi2)

    else: # reverse
        v2 = v0
        beta1 = beta1 - dt*beta1dot
        v1 = ((L3+L2*np.cos(beta1))*v2 + L3*L2*np.sin(beta1)*beta1dot)/(L2+L3*np.cos(beta1)) 
        psidot2 = (v1*np.sin(beta1) + L3*np.cos(beta1)*beta1dot)/(L3+L2*np.cos(beta1))
        psi2 = psi2 + dt*psidot2
        X2 = X2 + dt*v2*np.cos(psi2)
        Y2 = Y2 + dt*v2*np.sin(psi2)
        psi1 = psi2 + beta1
        X1 = X1 + dt*v1*np.cos(psi1)
        Y1 = Y1 + dt*v1*np.sin(psi1)
        beta0 = beta0 - dt*beta0dot
        psi0 = psi1 + beta0
        Xj0 = X1 + L1*np.cos(psi1)
        Yj0 = Y1 + L1*np.sin(psi1)   
        X0 = Xj0 + L0*np.cos(psi0)
        Y0 = Yj0 + L0*np.sin(psi0)
        
    # store state and control trajectory
    t_arr.append(dt*i)
    X0_arr.append(X0)
    Y0_arr.append(Y0)
    psi0_arr.append(psi0)
    beta0_arr.append(beta0)
    X1_arr.append(X1)
    Y1_arr.append(Y1)
    psi1_arr.append(psi1)
    beta1_arr.append(beta1)
    X2_arr.append(X2)
    Y2_arr.append(Y2)
    psi2_arr.append(psi2)
    beta1dot_arr.append(beta1dot)
    v1_arr.append(v1)
    v2_arr.append(v2)
    
    # store wheel positions
    if (i==0):
        axle0_pos = get_axle_position(X0,Y0,psi0,w)
        axle1_pos = get_axle_position(X1,Y1,psi1,w)
        axle2_pos = get_axle_position(X2,Y2,psi2,w)
    else:
        axle0_pos = np.vstack([axle0_pos, get_axle_position(X0,Y0,psi0,w)])
        axle1_pos = np.vstack([axle1_pos, get_axle_position(X1,Y1,psi1,w)])
        axle2_pos = np.vstack([axle2_pos, get_axle_position(X2,Y2,psi2,w)])
        
    # visualize
    Ni = 50
    if(i%Ni == 0):
        print "t = " + str(dt*i) 
        plt.clf()
        
        # frame
        plt.plot([X0_arr[i], X0_arr[i] - L0*np.cos(psi0_arr[i])], [Y0_arr[i], Y0_arr[i]-L0*np.sin(psi0_arr[i])], 'k-')
        plt.plot([X1_arr[i], X1_arr[i] + L1*np.cos(psi1_arr[i])], [Y1_arr[i], Y1_arr[i]+L1*np.sin(psi1_arr[i])], 'k-')
        plt.plot([X1_arr[i], X1_arr[i] - L2*np.cos(psi1_arr[i])], [Y1_arr[i], Y1_arr[i]-L2*np.sin(psi1_arr[i])], 'k-')
        plt.plot([X2_arr[i], X2_arr[i] + L3*np.cos(psi2_arr[i])], [Y2_arr[i], Y2_arr[i]+L3*np.sin(psi2_arr[i])], 'k-')
        #plt.plot(Xj0,Yj0,'*b')   
        
        # wheel traces
        plt.plot(axle0_pos[:,0],axle0_pos[:,2], 'C0')
        plt.plot(axle0_pos[:,1],axle0_pos[:,3], 'C0')
        plt.plot(axle1_pos[:,0],axle1_pos[:,2], 'C1')
        plt.plot(axle1_pos[:,1],axle1_pos[:,3], 'C1')
        plt.plot(axle2_pos[:,0],axle2_pos[:,2], 'C2')
        plt.plot(axle2_pos[:,1],axle2_pos[:,3], 'C2')
        
        # wheel markers
        plt.plot(axle0_pos[i][0:2],axle0_pos[i][2:4], 'k-o')
        plt.plot(axle1_pos[i][0:2],axle1_pos[i][2:4], 'k-o')
        plt.plot(axle2_pos[i][0:2],axle2_pos[i][2:4], 'k-o')
        
        plt.axis([-4, 4, -4, 4])
        plt.draw()
        plt.pause(0.0001)       


# misc ideas and leftovers
# beta1 tracks reference beta0(s) @ s1 = s0-(L1+L2), 
# saturate beta1prime (beta1dot = beta1prime*v0)

#    if (ctrl_strategy == 0): # no actuation
#        print "actuation deactivated" 
#        psidot1 = (v1*np.sin(beta0)-L0*beta0dot)/(L0 + L1*np.cos(beta0))
#        psidot2 = (v1*np.sin(beta1) - psidot1*L2*np.cos(beta1))/L3
#        psi2 = psi2 + dt*psidot2
#        beta1 = psi1 - psi2
#    else: # with actuatuion of beta1dot
#        print "actuation active, beta1dot = " + str(beta1dot) 
#        beta1 = beta1 + dt*beta1dot
#        psi2 = psi1-beta1
   