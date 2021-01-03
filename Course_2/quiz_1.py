import numpy as np

import modern_robotics as Mr
from math import pi

##################################
########### Giviens ##############
######### Question#1 #############
M=np.array([[1,0,0,3.73],
            [0,1,0,0],
            [0,0,1,2.73],
            [0,0,0,1]    ])
print("Question 1:\n" ,np.array2string(M, separator=',', suppress_small=True))
######### Question#2 #############
W_list=np.array([[0,0,1],#w1
                  [0,1,0],#w2
                  [0,1,0],#w3
                  [0,1,0],#w4
                  [0,0,0],#w5
                  [0,0,1]#w6      
                  ])

r_s_list=np.array([[1,0,0],#r1
                  [1,0,0],#r2
                  [2.73,0,-1],#r3
                  [3.73,0,0.73],#r4
                  [3.73,0,2.73],#r5
                  [3.73,0,2.73]#r6      
                  ])
V_s_list=np.array(np.cross(W_list,(-r_s_list)))
V_s_list[4,2]=1
S_list=np.array(np.concatenate((W_list,V_s_list),axis=1)).T
print("\n")
print("Question 2:\n", np.array2string(S_list, separator=','))

######### Question#3 #############
r_b_list=np.array([[-2.73,0,-2.73],#r1
                  [-2.73,0,-2.72],#r2
                  [0,0,-3.73],#r3
                  [-1,0,-2],#r4
                  [0,0,0],#r5
                  [0,0,0]#r6      
                  ])

V_b_list=np.array(np.cross(W_list,(-r_b_list)))
V_b_list[4,2]=1
B_list=np.array(np.concatenate((W_list,V_b_list),axis=1)).T

print("\n")
print("Question 3:\n", np.array2string(B_list, separator=','))
######### Question#4 #############
Theta_list=np.array([[-pi/2, pi/2, pi/3, -pi/4, 1, pi/6]])
Fk=Mr.FKinSpace(M,S_list,Theta_list)
Fk_s=np.around(Fk,2)
print("Question #4:\n",np.array2string(Fk_s,separator=','))
######### Question#4 #############
Fk=Mr.FKinBody(M,B_list,Theta_list)
Fk_b=np.around(Fk,2)
print("Question #5:\n",np.array2string(Fk_b,separator=','))
