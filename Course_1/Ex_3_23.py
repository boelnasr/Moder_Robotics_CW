import numpy as np 

import modern_robotics as Mr


###Givens####

r_s=np.array([2,-1,0])
r_b=np.array([2,-1.4,0])
w_s=np.array([0,0,2])
w_b=np.array([0,0,-2])


#######################
v_s=np.cross(w_s,-(r_s))
v_b=np.cross(w_b,-r_b)
Twist_s=np.concatenate((w_s,v_s))
Twist_b=np.concatenate((w_b,v_b))
Twist_s=np.array([Twist_s]).T
print(Twist_s)
print(Twist_b)