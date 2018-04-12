import numpy as np
from scipy.linalg import expm
#function M = forwardKinematicsUr3(theta)
def forwardKinematicsUr3(theta):
    a1=np.array([[0],[0],[1]])
    q1 = np.array([[0],[0],[0]])
    S1=rev_screw(a1,q1)

    a2=np.array([[0],[-1],[0]])
    q2 = np.array([[0],[0],[-.152]])
    S2=rev_screw(a2,q2)

    a3=np.array([[0],[-1],[0]])
    q3 = np.array([[0],[-.120],[.396]])
    S3=rev_screw(a3,q3)

    a4=np.array([[0],[1],[0]])
    q4 = np.array([[0],[-.027],[.609]])
    S4=rev_screw(a4,q4)

    a5=np.array([[0],[0],[1]])
    q5 = np.array([[0],[-0.110],[0.692]])
    S5=rev_screw(a5,q5)

    a6=np.array([[0],[-1],[0]])
    q6 = np.array([[0],[-.192],[.692]])
    S6=rev_screw(a6,q6)

    S = [S1,S2,S3,S4,S5,S6]


    Tinit = np.array([[1,0,0,0],[0,0,-1,-.192],[0,1,0,0.692],[0,0,0,1]])

    T_1in0 = Pose(S1,theta[0])
    T_2in0 = np.dot(T_1in0,Pose(S2,theta[1]))
    T_3in0 = np.dot(T_2in0,Pose(S3,theta[2]))
    T_4in0 = np.dot(T_3in0,Pose(S4,theta[3]))
    T_5in0 = np.dot(T_4in0,Pose(S5,theta[4]))
    T_6in0 = np.dot(T_5in0,Pose(S6,theta[5]))
    T_toolin0 = np.dot(T_6in0,Tinit)
    M = T_toolin0

    return M

#J = [S1 adjoint_matrix(T_1in0)*S2 adjoint_matrix(T_2in0)*S3];
#Jbod=inv(adjoint_matrix(Pose(S1,theta(1))*Pose(S2,theta(2))*Pose(S3,theta(3))*Tinit))*J;



def Pose(S,theta):
    M=expm(np.dot(bracket_twist(S),theta))
    return M

def rev_screw(a,q):
    M=screw(a,np.dot(-bracket_3x3(a),q));
    return M

def screw(W,V):
    S = np.zeros((6,1))
    S[0:3]=W
    S[3:6]=V
    #S = np.transpose(S)
    return S

def bracket_twist(S):
    M = np.zeros((4,4))
    M[0:3,0:3] = bracket_3x3(S[0:3,])
    M[0:3,[3]] = S[3:6]
    M[3] = [0,0,0,1]
    return M

def bracket_3x3(R):

    M = np.array([[0, -R[2], R[1]],[R[2],0,-R[0]],[-R[1],R[0],0]])
    return M



