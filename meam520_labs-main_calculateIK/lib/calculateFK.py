import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class FK():

    def __init__(self):
        """
        This is the dimension of the Panda Robot stated as global variable

        """

        # PANDA Dimensions in m
        # Offset along Z axis
        self.d1 = 0.333        # distance between frame 0 and frame 1
        self.d2 = 0            # distance between frame 1 and frame 2
        self.d3 = 0.316        # distance between frame 2 and frame 3
        self.d4 = 0            # distance between frame 3 and frame 4
        self.d5 = 0.384        # distance between frame 4 and frame 5
        self.d6 = 0            # distance between frame 5 and frame 6
        self.d7 = 0.210        # distance between frame 6 and frame 7

        # Offset along X axis
        self.a1 = 0.0     # offset between frame 0 and frame 1
        self.a2 = 0.0     # offset between frame 1 and frame 2
        self.a3 = 0.0825  # offset between frame 2 and frame 3
        self.a4 = -0.0825 # offset between frame 3 and frame 4
        self.a5 = 0.0     # offset between frame 4 and frame 5
        self.a6 = 0.088   # offset between frame 5 and frame 6
        self.a7 = 0.0     # offset between joint 6 and frame 7

        # SPONG convention for DH parameters
        self.alpha1 = -np.pi / 2.0
        self.alpha2 =  np.pi / 2.0
        self.alpha3 =  np.pi / 2.0
        self.alpha4 = -np.pi / 2.0
        self.alpha5 =  np.pi / 2.0
        self.alpha6 =  np.pi / 2.0
        self.alpha7 =  0.0

        # Joint limits
        self.lowerLim = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]).reshape((1, 7))    # Lower joint limits in radians ** This does not include gripper
        self.upperLim = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973]).reshape((1, 7))          # Upper joint limits in radians (grip in mm)

    def rot_z(self, theta ):
        # joint angle
        rot_z = np.matrix([
            [ np.cos(theta), -np.sin(theta), 0, 0],
            [ np.sin(theta),  np.cos(theta), 0, 0],
            [ 0,              0,             1, 0],
            [ 0,              0,             0, 1]
        ])

        return(rot_z)

    def rot_x(self, alpha ):
        # Link twist
        rot_alpha = np.matrix([
            [ 1, 0,              0,             0],
            [ 0, np.cos(alpha), -np.sin(alpha), 0],
            [ 0, np.sin(alpha),  np.cos(alpha), 0],
            [ 0,              0,             0, 1]
        ])

        return(rot_alpha)

    def trans_z(self, d):
        # Link length
        trans_z = np.matrix([[1, 0, 0, 0],
                             [0, 1, 0, 0],
                             [0, 0, 1, d],
                             [0, 0, 0, 1]])

        return(trans_z)

    def trans_x(self, a):
        # Link offset
        trans_x = np.matrix([[1, 0, 0, a],
                             [0, 1, 0, 0],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
        return(trans_x)

    def compute_A(self, theta, alpha, d, a):
        # A = Rot_z trans_z trans_x Rot_x
        # Traditional DH from Spong
        A = self.rot_z(theta) @ self.trans_z(d) @ self.trans_x(a) @ self.rot_x(alpha)
        return(A)

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions - 8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Your code starts from here
        Ai = self.compute_Ai(q)

        # Compute joint poses
        x = np.empty((8, 4)).reshape((8, 4))

        # Position of First Joint
        joint1_pos = np.array([0, 0, 0.141, 1]).reshape((1,4)) # in frame 0
        x[0, :] = joint1_pos

        # Position of Second Joint
        joint2_pos = np.array([0, 0, 0, 1]).reshape((4,1)) # in frame 1
        x[1, :] = np.transpose( Ai[0] @ joint2_pos )

        # Position of Third Joint
        joint3_pos = np.array([0, 0, 0.195, 1]).reshape((4,1)) # in frame 2
        x[2, :] = np.transpose( Ai[0] @ Ai[1] @ joint3_pos )

        # Position of Fourth Joint
        joint4_pos = np.array([0, 0, 0, 1]).reshape((4,1)) # in frame 3
        x[3, :] = np.transpose( Ai[0] @ Ai[1] @ Ai[2] @ joint4_pos )

        # Position of Fifth Joint
        joint5_pos = np.array([0, 0, 0.125, 1]).reshape((4,1)) # in frame 4
        x[4, :] = np.transpose( Ai[0] @ Ai[1] @ Ai[2] @ Ai[3] @ joint5_pos )

        # Position of Sixth Joint
        joint6_pos = np.array([0, 0, -0.015, 1]).reshape((4,1)) # in frame 5
        x[5, :] = np.transpose( Ai[0] @ Ai[1] @ Ai[2] @ Ai[3] @ Ai[4] @ joint6_pos)

        # Position of Seventh Joint
        joint7_pos = np.array([0, 0, 0.051, 1]).reshape((4,1))
        x[6, :] = np.transpose( Ai[0] @ Ai[1] @ Ai[2] @ Ai[3] @ Ai[4] @ Ai[5] @ joint7_pos )

        # Homogeneous transformation of end effector frame expressed in base frame
        T0e = Ai[0] @ Ai[1] @ Ai[2] @ Ai[3] @ Ai[4] @ Ai[5] @ Ai[6] @ Ai[7]

        x[7,:] = np.transpose( T0e[range(4),3] )

        # Outputs the 8x3 of the locations of each joint in the Base Frame
        jointPositions = x[:,0:3]

        # Your code ends here

        return jointPositions, T0e

    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
        return ()


    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # Frame 1 w.r.t Frame 0
        A1 = self.compute_A(q[0], self.alpha1, self.d1, self.a1)

        # Frame 2 w.r.t Frame 1
        A2 = self.compute_A(q[1], self.alpha2, self.d2, self.a2)

        # Frame 3 w.r.t Frame 2
        A3 = self.compute_A(q[2], self.alpha3, self.d3, self.a3)

        # Frame 4 w.r.t Frame 3
        A4 = self.compute_A(q[3], self.alpha4, self.d4, self.a4)

        # Frame 5 w.r.t Frame 4
        A5 = self.compute_A(q[4], self.alpha5, self.d5, self.a5)

        # Frame 6 w.r.t Frame 5
        A6 = self.compute_A(q[5], self.alpha6, self.d6, self.a6)

        # Frame 7 w.r.t Frame 6
        A7 = self.compute_A(q[6], self.alpha7, self.d7, self.a7)

        # Gripper w.r.t Frame 7
        # Note the rotation about z gives us a gripper frame aligned with the plane
        theta = -np.pi/4
        A8 = np.matrix([
            [ np.cos(theta), -np.sin(theta), 0, 0],
            [ np.sin(theta),  np.cos(theta), 0, 0],
            [ 0,              0,             1, 0],
            [ 0,              0,             0, 1]
        ])

        return([A1, A2, A3, A4, A5, A6, A7, A8])

    def print_results(self, joints, T0e):
        np.set_printoptions(precision = 2)
        print("SPONG CONVENTION")
        for i in range(joints.shape[0]):
            print("JOINT {0}: {1}".format(i, joints[i,:]))

        print("Transformation matrix from base to end effector")
        print(T0e)
        print("\n\n")

if __name__ == "__main__":
    np.set_printoptions(precision=4)

    fk = FK()

    q = np.array([0, 0 ,0, 0, 0, 0, 0, 0])
    joints2a, T0e = fk.forward(q)
    fk.print_results(joints2a, T0e)

    q = np.array([0, 0, 0, -np.pi/2, 0, np.pi/2, np.pi/4, 0])
    joints2b, T0e = fk.forward(q)
    fk.print_results(joints2b, T0e)
