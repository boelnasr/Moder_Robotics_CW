from modern_robotics import MatrixExp3
from modern_robotics import so3ToVec
from modern_robotics import Adjoint
import numpy as np

#question 5
Vb = np.array([[0, -.33, -.25], [.33, 0, -.12], [.25, .12, 0]])

a=so3ToVec(MatrixExp3(Vb))
print("\nQuestion 5:\n", np.array2string(np.around(a,decimals=3), separator=','), sep='')   
#question 6
Teb = [[0,-1,0,2], [1,0,0,3],[0,0,1,0],[0,0,0,1]]
r = .5
d = 1
F = [[-r/(2*d), r/(2*d)], [r/2, r/2]]

b=Adjoint(Teb)
print("\nQuestion 6:\n", np.array2string(np.around(b,decimals=3), separator=','), sep='')   
