import numpy as np
import modern_robotics as mr
import numpy.linalg as la
from math import pi
import math
############################################
######### question #1 ######################
F_s_1=np.array([0,0,0,2,0,0])
w_s_list_1=np.array([[0,0,1],
                    [0,0,1],
                    [0,0,1]])
r_s_list_1=np.array([[0,0,0],
                    [1,0,0],
                    [2,0,0]])
V_s_list_1=np.array(np.cross(w_s_list_1,(-r_s_list_1)))
S_list_1=np.array(np.concatenate((w_s_list_1,V_s_list_1),axis=1)).T
theta_list_1=np.array([0,pi/4,0])
J_s_1=mr.JacobianSpace(S_list_1,theta_list_1)
taulist_1=np.around(np.matmul(J_s_1.T,F_s_1),decimals=2)
print("Question 1:\n",np.array2string(taulist_1,separator=','))
######### question #2 ######################
w_b_list_2=np.array([[0,0,0],
                    [0,0,1],
                    [0,0,1],
                    [0,0,1]])
r_b_list_2=np.array([[0,0,0],
                    [1,0,0],
                    [2,0,0],
                    [3,0,0]])
V_b_list_2=np.array(np.cross(w_b_list_2,(-r_b_list_2)))
B_list_2=np.array(np.concatenate((w_b_list_2,V_b_list_2),axis=1)).T
theta_list_2=np.array([0,0,pi/2,-pi/2])
F_b_2=np.array([0,0,10,10,10,0])
J_b_2=mr.JacobianBody(B_list_2,theta_list_2)
taulist_2=np.around(np.matmul(J_b_2.T,F_b_2),decimals=2)
print("Question 2:\n",np.array2string(taulist_2,separator=','))
######### question #3 ######################
S_list_3=np.array([ [0,1,0],
                    [0,0,0],
                    [1,0,0],
                    [0,0,0],
                    [0,2,1],
                    [0,0,0]
                    ])
theta_list_3=np.array([pi/2,pi/2,1])
J_s_3=mr.JacobianSpace(S_list_3,theta_list_3)
print("Question 3:\n",np.array2string(np.around(J_s_3,decimals=2),separator=','))
######### question #4 ######################
B_list_4=np.array([[0,1,0,3,0,0],
                   [-1,0,0,0,3,0],
                   [0,0,0,0,0,1]] ).T
theta_list_4=np.array([pi/2, pi/2, 1])
J_b_4=mr.JacobianBody(B_list_4,theta_list_4)
J_b_4=np.around(J_b_4,decimals=2)
print("Question #4:\n",np.array2string(J_b_4,separator=','))
######### question #5 ######################
J_b_5 = np.array([[ 0.   ,  0.   ,  1.   , -0.105, -0.889,  0.   ],
                 [-1.   ,  0.   ,  0.   ,  0.   ,  0.006, -0.105],
                 [ 0.   ,  1.   ,  0.   ,  0.006,  0.   ,  0.889],
                 [ 0.   ,  0.   ,  1.   , -0.045, -0.844,  0.   ],
                 [-1.   ,  0.   ,  0.   ,  0.   ,  0.006,  0.   ],
                 [ 0.   ,  1.   ,  0.   ,  0.006,  0.   ,  0.   ],
                 [ 0.   ,  0.   ,  1.   ,  0.   ,  0.   ,  0.   ]]).T
J_b_5_v=J_b_5[3:]
A_5=np.matmul(J_b_5_v,J_b_5_v.T)
A_5=np.around(A_5,decimals=2)
w_5,v_5=la.eig(A_5)
maxId_5 = np.where(w_5 == np.amax(w_5))[0][0]
vecPAxis = v_5[maxId_5]
vecPAxis_off = np.around(vecPAxis, decimals=2)
print("\nQuestion 5:\n", np.array2string(vecPAxis_off, separator=','))
######### question #6 ######################
lenPAxis = math.sqrt(w_5[maxId_5])
lenPAxis_off = np.around(lenPAxis, decimals=2)
print("\nQuestion 6:\n", np.array2string(lenPAxis_off, separator=','))
####################################
