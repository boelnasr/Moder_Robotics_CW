import math
import numpy as np
import modern_robotics as mr

pi = math.pi

print("\n------ Question 1 ------")

density = 5600
rad_cyl = 0.02
len_cyl = 0.2
rad_sph = 0.1

mass_cyl = density * pi * len_cyl * rad_cyl**2
print("\nMass of Cylinder: ", mass_cyl, sep='')
Ixx_cyl = mass_cyl * (3 * rad_cyl**2 + len_cyl**2) / 12
Iyy_cyl = Ixx_cyl
Izz_cyl = mass_cyl * rad_cyl**2 / 2
RIMatrix_cyl = np.diag([Ixx_cyl, Iyy_cyl, Izz_cyl])
print("\nRIMatrix_cyl:\n", np.array2string(RIMatrix_cyl, separator=','), sep='')

mass_sph = density * 4/3 * pi * rad_sph**3
print("\nMass of Sphere: ", mass_sph, sep='')
Ixx_sph = mass_sph * 2/5 * rad_sph**2
Iyy_sph = Ixx_sph
Izz_sph = Ixx_sph
RIMatrix_sph = np.diag([Ixx_sph, Iyy_sph, Izz_sph])
print("\nRIMatrix_sph:\n", np.array2string(RIMatrix_sph, separator=','), sep='')

'''exploit Steiner's theorem'''
q1 = np.array([[0], [0], [rad_sph + len_cyl/2]])
RIMatrix_sph1 = RIMatrix_sph + mass_sph * (np.dot(q1.T, q1) * np.eye(3) - np.dot(q1, q1.T))
q2 = np.array([[0], [0], [-rad_sph - len_cyl/2]])
RIMatrix_sph2 = RIMatrix_sph + mass_sph * (np.dot(q2.T, q2) * np.eye(3) - np.dot(q2, q2.T))
print("\nRIMatrix_sph1:\n", np.array2string(RIMatrix_sph1, separator=','), sep='')
print("\nRIMatrix_sph2:\n", np.array2string(RIMatrix_sph2, separator=','), sep='')

RIMatrix_q1 = RIMatrix_cyl + RIMatrix_sph1 + RIMatrix_sph2
RIMatrix_q1_off = np.around(RIMatrix_q1, decimals=2)
print("\nQuestion 1:\n", np.array2string(RIMatrix_q1_off, separator=','), sep='')

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
joint_speed=np.array([0.2,0.2,0.2,0.2,0.2,0.2]).T
joint_accelration=np.array([0.1,0.1,0.1,0.1,0.1,0.1]).T
g=np.array([0,0,-9.81]).T
F_tip=np.array([0.1,0.1,0.1,0.1,0.1,0.1]).T
joint_forces=mr.InverseDynamics(thetalist,joint_speed,joint_accelration,g,F_tip,Mlist,Glist,Slist)

print("\nQuestion 5:\n", np.array2string(np.around(joint_forces,decimals=2), separator=','), sep='')    

