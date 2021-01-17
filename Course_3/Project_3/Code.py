import numpy as np
import modern_robotics as mr
from math import pi
import csv
from numpy.core.defchararray import array

M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67] 
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]
###########################################
################## Givens #################

thetalist_1_0=np.array(np.zeros(6))
thetalist_2_0=thetalist_1_0
thetalist_2_0[1]=-1
dtheta=np.array(np.zeros(6))
Ftip=np.array(np.zeros(6))
tau=np.array(np.zeros(6))
g=np.array([0,0,-9.81])

def simulation(thetalist,dthetalist,taulist,g,Ftip,Mlist,Glist,Slist,Number_of_Steps_per_second,time,filename):
    file1 = open(filename, "w", newline="")
    t=0
    while t<time:
        # update the csv
        csv.writer(file1).writerow([thetalist[0], thetalist[1], thetalist[2], thetalist[3], thetalist[4], thetalist[5]]) 
        dt=1/Number_of_Steps_per_second
        # retrieve joint accelerations
        ddthetalist = mr.ForwardDynamics(thetalist, dthetalist, taulist, g, Ftip, Mlist, Glist, Slist)
        # compute the next joint position and velocity
        thetalistNext, dthetalistNext = mr.EulerStep(thetalist, dthetalist, ddthetalist, dt)     
        # update 
        thetalist = thetalistNext
        dthetalist = dthetalistNext
        t+=dt
        
        #np.savetxt(filename, thetalist.T, delimiter=","'\n') # Saving to txt format
    
##########First simulation #################
simulation(thetalist_1_0,dtheta,tau,g,Ftip,Mlist,Glist,Slist,100,3,"sim1.csv")
##########First simulation #################
simulation(thetalist_2_0,dtheta,tau,g,Ftip,Mlist,Glist,Slist,100,5,"sim2.csv")

