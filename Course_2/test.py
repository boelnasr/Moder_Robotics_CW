import math
import numpy as np
from numpy import linalg as LA
import modern_robotics as mr

# global variables
pi = math.pi

# question 1
Fs_1 = np.array([0, 0, 0, 2, 0, 0])  # wrench on the tip
Slist_1 = np.array([[  0,  0,  1,  0,  0,  0],
                    [  0,  0,  1,  0, -1,  0],
                    [  0,  0,  1,  0, -2,  0]]).T
thetalist_1 = np.array([0, pi/4, 0])

Js_1 = mr.JacobianSpace(Slist_1, thetalist_1)
taulist_1 = np.dot(Js_1.T, Fs_1)
taulist_1_off = np.around(taulist_1, decimals=2)

print("\nQuestion 1:\n", np.array2string(taulist_1_off, separator=','))

# question  2
Fb_2 = np.array([  0,  0, 10, 10, 10,  0])
Blist_2 = np.array([[  0,  0,  1,  0,  4,  0],
                    [  0,  0,  1,  0,  3,  0],
                    [  0,  0,  1,  0,  2,  0],
                    [  0,  0,  1,  0,  1,  0]]).T
thetalist_2 = np.array([      0,      0,  pi/2, -pi/2])

Jb_2 = mr.JacobianBody(Blist_2, thetalist_2)
taulist_2 = np.dot(Jb_2.T, Fb_2)
taulist_2_off = np.around(taulist_2, decimals=2)

print("\nQuestion 2:\n", np.array2string(taulist_2_off, separator=','))

# question 3
Slist_3 = np.array([[ 0, 0, 1, 0, 0, 0],
                    [ 1, 0, 0, 0, 2, 0],
                    [ 0, 0, 0, 0, 1, 0]]).T
thetalist_3 = np.array([ pi/2, pi/2,    1])

Js_3 = mr.JacobianSpace(Slist_3, thetalist_3)
Js_3_off = np.around(Js_3, decimals=2)

print("\nQuestion 3:\n", np.array2string(Js_3_off, separator=','))

# question 4
Blist_4 = np.array([[  0,  1,  0,  3,  0,  0],
                    [ -1,  0,  0,  0,  3,  0],
                    [  0,  0,  0,  0,  0,  1]]).T
thetalist_4 = np.array([ pi/2, pi/2,    1])

Jb_4 = mr.JacobianBody(Blist_4, thetalist_4)
Jb_4_off = np.around(Jb_4, decimals=2)

print("\nQuestion 4:\n", np.array2string(Jb_4_off, separator=','))

# question 5
Jb_5 = np.array([[ 0.   ,  0.   ,  1.   , -0.105, -0.889,  0.   ],
                 [-1.   ,  0.   ,  0.   ,  0.   ,  0.006, -0.105],
                 [ 0.   ,  1.   ,  0.   ,  0.006,  0.   ,  0.889],
                 [ 0.   ,  0.   ,  1.   , -0.045, -0.844,  0.   ],
                 [-1.   ,  0.   ,  0.   ,  0.   ,  0.006,  0.   ],
                 [ 0.   ,  1.   ,  0.   ,  0.006,  0.   ,  0.   ],
                 [ 0.   ,  0.   ,  1.   ,  0.   ,  0.   ,  0.   ]]).T
Jv_5 = Jb_5[3:]

A_5 = np.dot(Jv_5, Jv_5.T)
w_5, v_5 = LA.eig(A_5)
maxId_5 = np.where(w_5 == np.amax(w_5))[0][0]

vecPAxis = v_5[maxId_5]
vecPAxis_off = np.around(vecPAxis, decimals=2)
print("\nQuestion 5:\n", np.array2string(vecPAxis_off, separator=','))

# question 6
lenPAxis = math.sqrt(w_5[maxId_5])
lenPAxis_off = np.around(lenPAxis, decimals=2)
print("\nQuestion 6:\n", np.array2string(lenPAxis_off, separator=','))