from modern_robotics import MatrixExp3
from modern_robotics import so3ToVec
from modern_robotics import Adjoint
from modern_robotics import TransInv
import numpy as np
import math
import modern_robotics as mr
#question 4
F=np.array([[-0.2,0.2,0.2,-0.2],[1,1,1,1],[-1,1,-1,1]])

F[:]=[x/4 for x in F]

theta_dot=np.array([-1.18,0.68,0.02,-0.52]).T

Vb=np.matmul(F,theta_dot)

print("\nQuestion 4:\n", np.array2string(np.around(Vb,decimals=3), separator=','), sep='')   
#question 5
wbz=Vb[0]
vbx=Vb[1]
vby=Vb[2]
phi=0
b=(vbx*math.sin(wbz)+vby*(math.cos(wbz)-1))/wbz
c=(vby*math.sin(wbz)+vbx*(-math.cos(wbz)+1))/wbz
dqp=np.array([wbz,b,c])

print("\nQuestion 5:\n", np.array2string(np.around(dqp,decimals=3), separator=','), sep='')   
#question 6
Tbe = [[0,-1,0,2], [1,0,0,3],[0,0,1,0],[0,0,0,1]]
Teb=TransInv(Tbe)
#print(Teb)
r = .5
d = 1
F = [[-r/(2*d), r/(2*d)], [r/2, r/2],[0,0]]
F6=[[0,0],[-r/(2*d), r/(2*d)], [r/2, r/2],[0,0],[0,0],[0,0]]
J_base=np.matmul(Adjoint(Tbe),F6)
print("\nQuestion 6:\n", np.array2string(np.around(J_base,decimals=3), separator=','), sep='')   
#question 7
w=np.array([0,0,1])
p=np.array([0,-3,0])
B=np.cross(p,w)
J_arm=mr.JacobianBody(B,[-math.pi/2])
print("\nQuestion 7:\n", np.array2string(np.around(J_arm,decimals=3), separator=','), sep='')   