import numpy as np 
import modern_robotics as mr
from math import pi 
from numpy import linalg as la 


###############################################
#####pramters intailization#####################
L1 = 0.425
L2 = 0.392
H1 = 0.089
H2 = 0.095
W1 = 0.109
W2 = 0.082

#end-effector transformation matrix at Home posetion 
M = np.array([  [-1, 0, 0, (L1+L2)],
                [0, 0, 1, (W1+W2)],
                [0, 1, 0, (H1-H2)],
                [0, 0, 0, 1]      ])
#desired end-effector Transformation
T_sd = np.array([[0,1,0,-0.5],
                [0,0,-1,0.1],
                [-1,0,0,0.1],
                [0,0,0,1]])
#allowable error
eomg = 0.001
ev = 0.0001
omega_list=np.array([[0,1,0],#w1
                  [0,0,1],#w2
                  [0,0,1],#w3
                  [0,0,1],#w4
                  [0,-1,0],#w5
                  [0,0,1]#w6      
                  ])
V_b_list=np.array([[W1+W2,0,L1+L2],#r1
                  [H2,-(L1+L2),0],#r2
                  [0,0,-3.73],#r3
                  [H2,-L2,0],#r4
                  [-W2,0,0],#r5
                  [0,0,0]#r6      
                  ])
B_list=np.array(np.concatenate((omega_list,V_b_list),axis=1)).T
thetalist0 = np.array([2.586,-6.283,6.283,6.283,-0.555,4.783])
def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    thetalist=thetalist0
    i=0
    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, \
                                                      thetalist)), T)))
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
          or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    
    max_i=20
    jointVecIter = thetalist
    while (err and i<max_i):
        thetalist = thetalist \
                    + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, \
                                                         thetalist)), Vb)
        
        jointVecIter = np.vstack([jointVecIter,thetalist])       #joining present joining angles to previous
        i = i + 1
        print("Iteration  ",i, " :")
        print("joint vector  :\n",thetalist)
        print("SE(3) end-effector config  : \n", mr.FKinBody(M, Blist, thetalist))
        Vb \
        = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, \
                                                       thetalist)), T)))
        print("error twist V_b : \n", Vb)                                               
        err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
              or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
        print("angular error magnitude ||omega_b|| : ", np.linalg.norm([Vb[0], Vb[1], Vb[2]]))
        print("linear error magnitude ||v_b|| : ", np.linalg.norm([Vb[3], Vb[4], Vb[5]]))
    
    np.savetxt("iterator.csv", jointVecIter, delimiter=",") # Saving to txt format
    return (thetalist, not err)

IKinBodyIterates(B_list, M, T_sd, thetalist0, eomg, ev) # Calling function