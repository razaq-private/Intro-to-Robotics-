import numpy as np
from math import pi


class FK():

    def __init__(self):

    # TODO: you may want to define geometric parameters here that will be
    # useful in computing the forward kinematics. The data you will need
    # is provided in the lab handout
        pass

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
        Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
        The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
        representing the end effector frame expressed in the
        world frame
        """

    # Your Lab 1 code starts here

        jointPositions = np.zeros((8,3))
        T0e = np.identity(4)


        a = [0, 0, 0, 0.0825, -0.0825, 0, 0.088, 0]
        alpha = [0, -pi/2, pi/2, pi/2, -pi/2, pi/2, pi/2, 0]
        d = [0.141, 0.192, 0, 0.316, 0, 0.384, 0, 0.21]


        A0 = np.array([[np.cos(0), -1*np.sin(0)*(np.cos(alpha[0])), np.sin(0)*(np.sin(alpha[0])), a[0]*(np.cos(q[0]))],
                        [np.sin(0), np.cos(0)*(np.cos(alpha[0])), -1*np.cos(0)*(np.sin(alpha[0])), a[0]*(np.sin(0))],
                        [0, np.sin(alpha[0]), np.cos(alpha[0]), d[0]],
                        [0, 0, 0, 1]])

        A1 = np.array([[np.cos(q[0]), -1*np.sin(q[0])*np.cos(alpha[1]), np.sin(q[0])*np.sin(alpha[1]),a[1]*np.cos(q[0])],
                        [np.sin(q[0]), np.cos(q[0])*(np.cos(alpha[1])),-1*(np.cos(q[0]))*np.sin(alpha[1]),a[1]*np.sin(q[0])],
                        [0, np.sin(alpha[1]), np.cos(alpha[1]), d[1]],
                        [0, 0, 0, 1]])

        A2 = np.array([[np.cos(q[1]), -1*np.sin(q[1])*(np.cos(alpha[2])), np.sin(q[1])*np.sin(alpha[2]),a[2]*np.cos(q[1])],
                        [np.sin(q[1]), np.cos(q[1])*(np.cos(alpha[2])),-1*(np.cos(q[1]))*(np.sin(alpha[2])),a[2]*(np.sin(q[1]))],
                        [0, np.sin(alpha[2]), np.cos(alpha[2]), d[2]],
                        [0, 0, 0, 1]])

        A3 = np.array([[np.cos(q[2]), -1*np.sin(q[2])*(np.cos(alpha[3])), np.sin(q[2])*(np.sin(alpha[3])),a[3]*(np.cos(q[2]))],
                        [np.sin(q[2]), np.cos(q[2])*(np.cos(alpha[3])),-1*(np.cos(q[2]))*(np.sin(alpha[3])),a[3]*(np.sin(q[2]))],
                        [0, np.sin(alpha[3]), np.cos(alpha[3]), d[3]],
                        [0, 0, 0, 1]])

        A4 = np.array([[np.cos(q[3]), -1*np.sin(q[3])*(np.cos(alpha[4])), np.sin(q[3])*(np.sin(alpha[4])),a[4]*(np.cos(q[3]))],
                        [np.sin(q[3]), np.cos(q[3])*(np.cos(alpha[4])),-1*(np.cos(q[3]))*(np.sin(alpha[4])),a[4]*(np.sin(q[3]))],
                        [0, np.sin(alpha[4]), np.cos(alpha[4]), d[4]],
                        [0, 0, 0, 1]])

        A5 = np.array([[np.cos(q[4]), -1*np.sin(q[4])*(np.cos(alpha[5])), np.sin(q[4])*(np.sin(alpha[5])),a[5]*(np.cos(q[4]))],
                        [np.sin(q[4]), np.cos(q[4])*(np.cos(alpha[5])),-1*(np.cos(q[4]))*(np.sin(alpha[5])),a[5]*(np.sin(q[4]))],
                        [0, np.sin(alpha[5]), np.cos(alpha[5]), d[5]],
                        [0, 0, 0, 1]])

        A6 = np.array([[np.cos(q[5]), -1*np.sin(q[5])*(np.cos(alpha[6])), np.sin(q[5])*(np.sin(alpha[6])),a[6]*np.cos(q[5])],
                        [np.sin(q[5]), np.cos(q[5])*(np.cos(alpha[6])),-1*np.cos(q[5])*(np.sin(alpha[6])),a[6]*(np.sin(q[5]))],
                        [0, np.sin(alpha[6]), np.cos(alpha[6]), d[6]],
                        [0, 0, 0, 1]])

        Ae = np.array([[np.cos(q[6]), -1*(np.sin(q[6]))*(np.cos(alpha[7])), np.sin(q[6])*(np.sin(alpha[7])),a[7]*(np.cos(q[6]))],
                        [np.sin(q[6]), np.cos(q[6])*(np.cos(alpha[7])),-1*(np.cos(q[6]))*(np.sin(alpha[7])),a[7]*(np.sin(q[6]))],
                        [0, np.sin(alpha[7]), np.cos(alpha[7]), d[7]],
                        [0, 0, 0, 1]])



        A01 = np.matmul(A0,A1)
        A02 = np.matmul(A01,A2)
        A03 = np.matmul(A02,A3)
        A04 = np.matmul(A03,A4)
        A05 = np.matmul(A04,A5)
        A06 = np.matmul(A05,A6)
        A0e = np.matmul(A06,Ae)

        #transform some of the matrices in order to account for the offsets
        translate2 = [ [1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0.195],
                        [0,0,0,1]]
        A02 = np.matmul(A02, translate2)

        translate4 = [ [1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0.125],
                        [0,0,0,1]]
        A04 = np.matmul(A04,translate4)

        translate5 = [ [1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,-0.015],
                        [0,0,0,1]]
        A05 = np.matmul(A05,translate5)

        translate6 = [ [1,0,0,0],
                        [0,1,0,0],
                        [0,0,1,0.051],
                        [0,0,0,1]]
        A06 = np.matmul(A06,translate6)



        pos0 = A0[:,3][0:3]
        pos1 = A01[:,3][0:3]
        pos2 = A02[:,3][0:3]
        pos3 = A03[:,3][0:3]
        pos4 = A04[:,3][0:3]
        pos5 = A05[:,3][0:3]
        pos6 = A06[:,3][0:3]
        pos7 = A0e[:,3][0:3]




        jointPositions = np.array([pos0, pos1, pos2, pos3, pos4, pos5, pos6, pos7])
        T0e =  A0e




    # Your code ends here
    # everytime you multiply the matrix you should get the joint from it


        return jointPositions, T0e
    # feel free to define additional helper methods to modularize your solution for lab 1


    # This code is for Lab 2, you can ignore it ofr Lab 1
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
        world frame

        """
    # STUDENT CODE HERE: This is a function needed by lab 2

        return()

    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
        necessarily located at the joint locations
        """
    # STUDENT CODE HERE: This is a function needed by lab 2

        return()

if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    q = np.array([0,0,0,-pi/2,0,pi/2,pi/4])

    joint_positions, T0e = fk.forward(q)

    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)
