import numpy as np
from calculateFK import FK

def calcJacobian(q_in):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """
    """
    This is the dimension of the Panda Robot stated as global variable
    """
    # PANDA Dimensions in m
    # Offset along Z axis
    d1 = 0.333        # distance between frame 0 and frame 1
    d2 = 0            # distance between frame 1 and frame 2
    d3 = 0.316        # distance between frame 2 and frame 3
    d4 = 0            # distance between frame 3 and frame 4
    d5 = 0.384        # distance between frame 4 and frame 5
    d6 = 0            # distance between frame 5 and frame 6
    d7 = 0.210        # distance between frame 6 and frame 7
    # Offset along X axis
    a1 = 0.0     # offset between frame 0 and frame 1
    a2 = 0.0     # offset between frame 1 and frame 2
    a3 = 0.0825  # offset between frame 2 and frame 3
    a4 = -0.0825 # offset between frame 3 and frame 4
    a5 = 0.0     # offset between frame 4 and frame 5
    a6 = 0.088   # offset between frame 5 and frame 6
    a7 = 0.0     # offset between joint 6 and frame 7
    # SPONG convention for DH parameters
    alpha1 = -np.pi / 2.0
    alpha2 =  np.pi / 2.0
    alpha3 =  np.pi / 2.0
    alpha4 = -np.pi / 2.0
    alpha5 =  np.pi / 2.0
    alpha6 =  np.pi / 2.0
    alpha7 =  0.0

    A0 = np.identity(4)
    A1 = computeA(q_in[0], alpha1, a1, d1)
    A2 = computeA(q_in[1], alpha2, a2, d2)
    A3 = computeA(q_in[2], alpha3, a3, d3)
    A4 = computeA(q_in[3], alpha4, a4, d4)
    A5 = computeA(q_in[4], alpha5, a5, d5)
    A6 = computeA(q_in[5], alpha6, a6, d6)
    A7 = computeA(q_in[6]-np.pi/4.0, alpha7, a7, d7)
    
    A01 = A0 @ A1
    A02 = A01 @ A2
    A03 = A02 @ A3
    A04 = A03 @ A4
    A05 = A04 @ A5
    A06 = A05 @ A6
    A07 = A06 @ A7

    R00_z = z_hat(A0)
    R01_z = z_hat(A01)
    R02_z = z_hat(A02)
    R03_z = z_hat(A03)
    R04_z = z_hat(A04)
    R05_z = z_hat(A05)
    R06_z = z_hat(A06)
    R07_z = z_hat(A07)
    
    """"
    R01_skew = skew(R01_z)
    R02_skew = skew(R02_z)
    R03_skew = skew(R03_z)
    R04_skew = skew(R04_z)
    R05_skew = skew(R05_z)
    R04_skew = skew(R06_z)
    R07_skew = skew(R07_z)
    """

    d_00 = distance(A0)
    d_01 = distance(A01)
    d_02 = distance(A02)
    d_03 = distance(A03)
    d_04 = distance(A04)
    d_05 = distance(A05)
    d_06 = distance(A06)
    d_07 = distance(A07)

    #q_in will be used for the theta part of the rotation matrices 
    J = np.zeros((6, 7))
    
    #print(R01_z.T )
    #print(d_01.T-d_00.T)
    #crosstest = np.cross(R00_z.T,(d_01.T-d_00.T))
    #print(crosstest.T)
    
    Jv1 = np.cross(R00_z.T, (d_07.T-d_00.T))
    Jv2 = np.cross(R01_z.T, (d_07.T-d_01.T))
    Jv3 = np.cross(R02_z.T, (d_07.T-d_02.T))
    Jv4 = np.cross(R03_z.T, (d_07.T-d_03.T))
    Jv5 = np.cross(R04_z.T, (d_07.T-d_04.T))
    Jv6 = np.cross(R05_z.T, (d_07.T-d_05.T))
    Jv7 = np.cross(R06_z.T, (d_07.T-d_06.T))
 

    Jw1 = R00_z 
    Jw2 = R01_z
    Jw3 = R02_z
    Jw4 = R03_z 
    Jw5 = R04_z 
    Jw6 = R05_z 
    Jw7 = R06_z 
    

    Jv = np.hstack((Jv1.T, Jv2.T, Jv3.T, Jv4.T, Jv5.T, Jv6.T, Jv7.T))
    Jw = np.hstack((Jw1, Jw2, Jw3, Jw4, Jw5, Jw6, Jw7))

    J = np.vstack((Jv, Jw))
    Jvnew = J[0:3,:]
    print(Jvnew)
    print("----JV----\n", Jvnew[:, 0:2])
    print("---J----\n", np.hstack((Jvnew[:,0:], np.zeros((3,5)))))
    ## STUDENT CODE GOES HERE

    f_att = np.zeros((3,7))
   
    print(f_att[:,1])
    print(f_att[:,0])
    
    



    return J
#take the last column of the rotation matrices



def computeA (theta, alpha, a, d): 
    A = np.array([ [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                   [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)], 
                   [0, np.sin(alpha), np.cos(alpha), d], 
                   [0, 0, 0, 1]])
    return A

def z_hat(R): 
    R_z = np.array([[R[0][2]], [R[1][2]], [R[2][2]]]) 
    return R_z

def distance(Ai): 
    dis = np.array([[Ai[0][3]], [Ai[1][3]], [Ai[2][3]]]) 
    return dis 

def skew(R_z): 
    R_skew = np.array([ [0, -R_z[2], R_z[1]],
                        [R_z[2], 0, -R_z[0]], 
                        [-R_z[1], R_z[0], 0]  ])
    return R_skew



if __name__ == '__main__':
    q= np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4])
    print(np.round(calcJacobian(q),3))




