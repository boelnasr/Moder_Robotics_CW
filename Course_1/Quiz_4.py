import numpy as np 


import modern_robotics as Mr


######Givens:######

T_sa=np.array([[0,-1,0,0] ,
                [0,0,-1,0] ,
                [1,0,0,1] ,
                [0,0,0,1]      ])
T_sb=np.array([ [1,0,0,0],
                [0,0,1,2] ,
                [0,-1,0,0] ,
                [0,0,0,1]      ])

###############Question #1###########
print("Question #1\n")
print(T_sa)
###############Question #2###########
print("Question #2\n")
T_bs=Mr.TransInv(T_sb)
print(T_bs)
###############Question #3###########
print("Question #3\n")
#T_ab=T_as*T_sb
T_as=Mr.TransInv(T_sa)
T_ab=np.dot(T_as,T_sb)
print(T_ab)
###############Question #5###########
print("Question #5\n")
P_p=np.array([[1,2,3,1]]).T
P_s=np.matmul(T_sb,P_p)
print(P_s)
###############Question #7###########
print("Question #7\n")
##V_a=adj_as*V_s
Adt_as=Mr.Adjoint(T_as)
V_s=np.array([[3,2,1,-1,-2,-3]]).T
V_a=np.matmul(Adt_as,V_s)
print(V_a)
###############Question #8###########
print("Question #8\n")
MatrixLog_T_sa = Mr.MatrixLog6(T_sa)
MatrixLog_T_sa_vec = Mr.se3ToVec(MatrixLog_T_sa)
theta = Mr.AxisAng6(MatrixLog_T_sa_vec)[1]
print(theta)
###############Question #9###########
print("Question #9\n")
V=np.array([[0,1,2,3,0,0]]).T
S=Mr.VecTose3(V)
Matrix_exponential=Mr.MatrixExp6(S)
print(np.around(Matrix_exponential,2))
###############Question #10###########
print("Question #10\n")
F_b = np.array([1, 0, 0, 2, 1, 0])
Ad_T_bs = Mr.Adjoint(T_bs)
Ad_T_bs_trans = np.transpose(Ad_T_bs)
F_s = np.dot(Ad_T_bs_trans, F_b)
print("\nQuestion 10:\n", np.array2string(F_s, separator=',')) # question 10
###############Question #11###########
print("Question #11\n")
T=np.array([[0,-1,0,3],
            [1,0,0,0],
            [0,0,1,1],
            [0,0,0,1]])
invT=Mr.TransInv(T)
print(invT)
###############Question #12###########
print("Question #12\n")
Twist=np.array([[1,0,0,0,2,3]]).T
se3mat=Mr.VecTose3(Twist)
print(se3mat)
###############Question #13###
print("Question #13\n")
p=np.array([0,0,2])
h=1
vec_s=np.array([1,0,0])
S=Mr.ScrewToAxis(p,vec_s,h)
print(S)
###############Question #14###
print("Question #14\n")
Matrix_Log=np.array([[0,-1.5708,0,2.3562],
                    [1.5708,0,0,-2.3562],
                    [0,0,0,1],
                    [0,0,0,0]])
T=Mr.MatrixExp6(Matrix_Log)
#T=np.around(T,0)
print(T)
###############Question #15###
print("Question #15\n")
T=np.array([[0,-1,0,3],
            [1,0,0,0],
            [0,0,1,1],
            [0,0,0,1]])
             
se3mat =Mr.MatrixLog6(T)
print(se3mat)

