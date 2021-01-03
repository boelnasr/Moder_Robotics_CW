import numpy as np 
import modern_robotics as mr
from math import pi 
from numpy import linalg as la 

def function_1(theta):
    return np.array([theta[0]**2-9, theta[1]**2-4])

def jacobian_1(theta):
    return np.array([[2*theta[0], 0], [0, 2*theta[1]]])

count = 0       # iteration counter
theta = [1, 1]  # initial guess
while(count < 2):
    count = count + 1
    theta = theta - np.dot(la.inv(jacobian_1(theta)), function_1(theta))

theta = np.around(theta, decimals=2)
print("\nQuestion 1:\n", np.array2string(theta, separator=','))
###############################################################
################# Question #2 #################################
Blist = np.array([[ 0, 0, 1, 0, 3, 0],
                  [ 0, 0, 1, 0, 2, 0],
                  [ 0, 0, 1, 0, 1, 0]]).T
M = np.array([[ 1, 0, 0, 3],
              [ 0, 1, 0, 0],
              [ 0, 0, 1, 0],
              [ 0, 0, 0, 1]])
T = np.array([[-0.585, -0.811, 0, 0.076],
              [ 0.811, -0.585, 0, 2.608],
              [ 0    ,  0    , 1, 0.   ],
              [ 0    ,  0.   , 0, 1.   ]])
thetalist0 = np.array([pi/4, pi/4, pi/4])
eomg = 0.001
ev = 0.0001
thetalist , sucsess=mr.IKinBody(Blist,M,T,thetalist0,eomg,ev)
thetalist = np.around(thetalist, decimals=2)
print("\nQuestion 2:\n", np.array2string(thetalist, separator=','))