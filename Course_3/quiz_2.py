import math
import numpy as np
import modern_robotics as mr
pi = math.pi
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
thetalist=np.array([0,pi/6,pi/4,pi/3,pi/2,(2*pi/3)]).T
dthetalist=np.array([0.2,0.2,0.2,0.2,0.2,0.2]).T
ddthetalist=np.array([0.1,0.1,0.1,0.1,0.1,0.1]).T
g=np.array([0,0,-9.81]).T
F_tip=np.array([0.1,0.1,0.1,0.1,0.1,0.1]).T
taulist=np.array([0.0128,-41.1477,-3.7809,0.0323,0.037,0.1034]).T
#joint_forces=mr.InverseDynamics(thetalist,joint_speed,joint_accelration,g,F_tip,Mlist,Glist,Slist)
 
Mass_Matrix = mr.MassMatrix(thetalist,Mlist,Glist,Slist)
print("\nQuestion 1:\n", np.array2string(np.around(Mass_Matrix,decimals=2), separator=','), sep='')   
c =mr.VelQuadraticForces(thetalist,dthetalist,Mlist,Glist,Slist)
print("\nQuestion 2:\n", np.array2string(np.around(c,decimals=2), separator=','), sep='')   
grav = mr.GravityForces(thetalist,g,Mlist,Glist,Slist)
print("\nQuestion 3:\n", np.array2string(np.around(grav,decimals=2), separator=','), sep='')   
JTFtip = mr.EndEffectorForces(thetalist,F_tip,Mlist,Glist,Slist)
print("\nQuestion 4:\n", np.array2string(np.around(JTFtip,decimals=2), separator=','), sep='')   
ddthetalist = mr.ForwardDynamics(thetalist,dthetalist,taulist,g,F_tip,Mlist,Glist,Slist)
print("\nQuestion 5:\n", np.array2string(np.around(ddthetalist,decimals=2), separator=','), sep='')   
