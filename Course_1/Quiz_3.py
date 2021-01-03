import numpy as np 

import modern_robotics as Mr


############################
import math

x = 2
e_to_2 = 0
for i in range(5):
    e_to_2 += x**i/math.factorial(i)
    
print(e_to_2)
###############Question #1###########
R_sa=np.array([[0,1,0],[0,0,1],[1,0,0]])
R_sb=np.array([[1,0,0],[0,0,1],[0,-1,0]])
print("Question #1\n")
print(R_sa)
###############Question #2###########
print("Question #2\n")
invR_sb=Mr.RotInv(R_sb)
print(invR_sb)
###############Question #3###########
print("Question #3\n")
invR_sa=Mr.RotInv(R_sa)
R_ab=np.matmul(invR_sa,R_sb)
print(R_ab)
###############Question #5 ###########
print("Question #5\n")
p_b=[1,2,3]
p=np.matmul(R_sb,p_b)
print(p)
###############Question #7 ###########
print("Question #7\n")
w= np.array([3,2,1]).T
w_a=np.matmul(invR_sa,w)
print(w_a)
###############Question #8 ###########
print("Question #8\n")
w= np.array([3,2,1]).T
w_a=np.matmul(invR_sa,w)
print(w_a)
###############Question #9###########
print("Question #9\n")
omg=[1,2,0]
so3mat = Mr.VecToso3(omg)
print(so3mat)
