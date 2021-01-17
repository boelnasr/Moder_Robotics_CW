import math
import numpy as np
import modern_robotics as mr
pi = math.pi

Tf=5
t=3
N=10
s = mr.QuinticTimeScaling(Tf,t)
print("\nQuestion 5:\n", np.around(s,decimals=2))  

X_start=np.array([  [1,0,0,0],
                    [0,1,0,0],
                    [0,0,1,0],
                    [0,0,0,1] ])
X_end=np.array([    [0,0,1,1],
                    [1,0,0,2],
                    [0,1,0,3],
                    [0,0,0,1] ])
Tf=10
traj = mr.ScrewTrajectory(X_start,X_end,Tf,N,3)
print("\nQuestion 6:\n", np.array2string(np.around(traj[8],decimals=3), separator=','), sep='')   
traj = mr.CartesianTrajectory(X_start,X_end,Tf,N,5)
print("\nQuestion 7:\n", np.array2string(np.around(traj[8],decimals=3), separator=','), sep='')   
