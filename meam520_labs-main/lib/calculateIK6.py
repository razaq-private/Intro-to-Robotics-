#Francesca's inverse kinematics
import numpy as np
from math import pi
import math
#from calculateFK import FK

class IK:
    """
    Solves the 6 DOF (joint 5 fixed) IK problem for panda robot arm
    """
    # offsets along x direction
    a1 = 0
    a2 = 0
    a3 = 0.0825
    a4 = 0.0825
    a5 = 0
    a6 = 0.088
    a7 = 0

    # offsets along z direction
    d1 = 0.333
    d2 = 0
    d3 = 0.316
    d4 = 0
    d5 = 0.384
    d6 = 0
    d7 = 0.210

    # This variable is used to express an arbitrary joint angle
    Q0 = 0.123


    def panda_ik(self, target):
        """
        Solves 6 DOF IK problem given physical target in x, y, z space
        Args:
            target: dictionary containing:
                'R': numpy array of the end effector pose relative to the robot base
                't': numpy array of the end effector position relative to the robot base

        Returns:
             q = nx7 numpy array of joints in radians (q5: joint 5 angle should be 0)
        """
        q = []

        # Student's code goes in between:

        # offsets along x direction
        a1 = 0
        a2 = 0
        a3 = 0.0825
        a4 = 0.0825
        a5 = 0
        a6 = 0.088
        a7 = 0

        # offsets along z direction
        d1 = 0.333
        d2 = 0
        d3 = 0.316
        d4 = 0
        d5 = 0.384
        d6 = 0
        d7 = 0.210

        #held constant
        q5 = 0

        # This variable is used to express an arbitrary joint angle
        Q0 = 0.123

        wrist_pos = []
        #get orientation and position
        eepose = target['R']
        eeposition = target['t'].reshape(-1,1)
        #get wrist position
        eeposetranspose = np.transpose(eepose)
        eepositioninv = np.matmul(-eeposetranspose,eeposition)
        wrist_pos = eepositioninv + d1*np.matmul(eeposetranspose, [[0],[0],[1]])

        joints_467 = []
        #theta7 solution 1
        if eeposition[0]==0 and eeposition[1] == 0:
            q7 = Q0
        else:
            q7 = 5*pi/4 - math.atan2(wrist_pos[1],wrist_pos[0])
            if q7 >= 2.8973:
                q7 = q7 - pi
            if q7 <=-2.8973:
                q7 = q7 + pi

        #get distance from frame 5 to frame 2
        t7tot6 = [[np.cos((7*np.pi/4+ np.pi/2)-q7), np.cos((5*np.pi/4 + np.pi/2)-q7), 0, 0], [np.cos((np.pi/4+ np.pi/2)-q7), np.cos((7*np.pi/4 + np.pi/2)-q7), 0, 0], [0, 0, 1, d7], [0, 0, 0, 1]]
        o2too6 = np.matmul(t7tot6,np.append(wrist_pos, [1]))
        o2too5 = o2too6 + [a6,0,0,1]

        #link lengths
        arm1 = np.sqrt(d5**2 + a4**2)
        arm2 = np.sqrt(d3**2 + a3**2)

        #theta6 and theta4 solution 1
        theta2 = np.arccos((o2too5[0]**2 + o2too5[2]**2 - arm1**2 - arm2**2)/(2*arm1*arm2))
        theta1 = math.atan2(o2too5[2],o2too5[0]) - math.atan2(arm2*np.sin(theta2),arm1 + arm2*np.cos(theta2))
        q6 = theta1 + np.arctan(a3/d5) - pi/2
        q4 = theta2 + np.arctan(d3/a3) + np.arctan(d5/a3) - pi

        #append twice for solving orienttation
        joints_467.append([q4,q6,q7])
        joints_467.append([q4,q6,q7])

        #theta 6 and theta4 solution 2
        theta2 = -np.arccos((o2too5[0]**2 + o2too5[2]**2 - arm1**2 - arm2**2)/(2*arm1*arm2))
        theta1 = math.atan2(o2too5[2],o2too5[0]) - math.atan2(arm2*np.sin(theta2),arm1 + arm2*np.cos(theta2))
        q6 = theta1 + np.arctan(a3/d5) - pi/2
        q4 = theta2 + np.arctan(d3/a3) + np.arctan(d5/a3) - pi

        #append twice for solving orienttation
        joints_467.append([q4,q6,q7])
        joints_467.append([q4,q6,q7])

        #theta7 solution 2
        if eeposition[0]==0 and eeposition[1] == 0:
            q7 = Q0
        else:
            if q7<0:
                q7 = q7 + pi
            if q7>0:
                q7 = q7 - pi
        #calculate distance from frame 5 to frame 2
        t7tot6 = [[np.cos((7*np.pi/4+ np.pi/2)-q7), np.cos((5*np.pi/4 + np.pi/2)-q7), 0, 0], [np.cos((np.pi/4+ np.pi/2)-q7), np.cos((7*np.pi/4 + np.pi/2)-q7), 0, 0], [0, 0, 1, d7], [0, 0, 0, 1]]
        o2too6 = np.matmul(t7tot6,np.append(wrist_pos, [1]))
        o2too5 = o2too6 + [a6,0,0,1]


        #theta 4 and theta 6 solution 3
        theta2 = np.arccos((o2too5[0]**2 + o2too5[2]**2 - arm1**2 - arm2**2)/(2*arm1*arm2))
        theta1 = math.atan2(o2too5[2],o2too5[0]) - math.atan2(arm2*np.sin(theta2),arm1 + arm2*np.cos(theta2))
        q6 = theta1 + np.arctan(a3/d5) - pi/2
        q4 = theta2 + np.arctan(d3/a3) + np.arctan(d5/a3) - pi

        #append twice for solving orientation
        joints_467.append([q4,q6,q7])
        joints_467.append([q4,q6,q7])

        #theta 4nad theta6 solution 4
        theta2 = -np.arccos((o2too5[0]**2 + o2too5[2]**2 - arm1**2 - arm2**2)/(2*arm1*arm2))
        theta1 = math.atan2(o2too5[2],o2too5[0]) - math.atan2(arm2*np.sin(theta2),arm1 + arm2*np.cos(theta2))
        q6 = theta1 + np.arctan(a3/d5) - pi/2
        q4 = theta2 + np.arctan(d3/a3) + np.arctan(d5/a3) - pi

        #append twice for solving orienttation
        joints_467.append([q4,q6,q7])
        joints_467.append([q4,q6,q7])

        joints_123 = []
        for i in range(len(joints_467)):
            r3tornew3 = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
            r4tor3 = [[np.cos(joints_467[i][0]), 0, -np.sin(joints_467[i][0])], [np.sin(joints_467[i][0]), 0, np.cos(joints_467[i][0])], [0, -1, 0]]
            r5tor4 = [[np.cos(q5), 0, np.sin(q5)], [np.sin(q5), 0, -np.cos(q5)], [0, 1, 0]]
            r6tor5 = [[np.cos(joints_467[i][1]), 0, np.sin(joints_467[i][1])], [np.sin(joints_467[i][1]), 0, -np.cos(joints_467[i][1])], [0, 1, 0]]
            r7tor6 = [[np.cos((7*np.pi/4+ np.pi/2)-joints_467[i][2]), np.cos((5*np.pi/4 + np.pi/2)-joints_467[i][2]),0], [np.cos((np.pi/4+ np.pi/2)-joints_467[i][2]), np.cos((7*np.pi/4 + np.pi/2)-joints_467[i][2]), 0], [0, 0, 1]]
            
            R = np.matmul(r3tornew3, np.matmul(r4tor3,np.matmul(r5tor4, np.matmul(r6tor5,r7tor6))))
            r0tornew = np.matmul(eepose, np.transpose(R))
            r0tornew[2][2] = np.round(r0tornew[2][2],7)
            q2 = np.arccos(r0tornew[2][2])
            if  (i % 2) == 0:
                q2 = q2
            else:
                q2 = -q2
            if q2 == 0:
                q1 = Q0
                q3 = Q0
                joints_123.append([q1,q2,q3])
            else:
                if joints_467[i][2] == Q0:
                    q1 = Q0
                    cosq3 = r0tornew[2][0]/-np.sin(q2)
                    sinq3 = r0tornew[2][1]/np.sin(q2)
                    q3 = math.atan2(sinq3,cosq3)
                    joints_123.append([q1,q2,q3])
                else:
                    cosq3 = r0tornew[2][0]/-np.sin(q2)
                    sinq3 = r0tornew[2][1]/np.sin(q2)
                    q3 = math.atan2(sinq3,cosq3)
                    cosq1 = r0tornew[0][2]/np.sin(q2)
                    sinq1 = r0tornew[1][2]/np.sin(q2)
                    q1 = math.atan2(sinq1,cosq1)
                    joints_123.append([q1,q2,q3])

        joint5 = []
        for i in range(len(joints_123)):
            joint5.append([0])

        qnolims = np.concatenate((joints_123, [[i[0]] for i in joints_467], joint5, [[i[1]] for i in joints_467], [[i[2]] for i in joints_467]), axis =1)
        qnolims = np.unique(qnolims, axis=0)

        q = []
        for row in qnolims:
            if row[0] > -2.8973 and row[0] < 2.8973 and row[1] > -1.7628 and row[1] < 1.7628 and row[2] > -2.8973 and row[2] < 2.8973 and row[3] > -3.0718 and row[3] < -0.0698 and row[5] > -0.0175 and row[5] < 3.7525 and row[6] > -2.8973 and row[6] < 2.8973:
                q.append(row) 
        if len(q) == 0:
            q = np.array([]).reshape(0,7)
        q = np.array(q)

        # Student's code goes in between:

        ## DO NOT EDIT THIS PART
        # This will convert your joints output to the autograder format
        q = self.sort_joints(q)
        ## DO NOT EDIT THIS PART
        return q
    def sort_joints(self, q, col=0):
        """
        Sort the joint angle matrix by ascending order
        Args:
            q: nx7 joint angle matrix
        Returns:
            q_as = nx7 joint angle matrix in ascending order
        """
        if col != 7:
            q_as = q[q[:, col].argsort()]
            for i in range(q_as.shape[0]-1):
                if (q_as[i, col] < q_as[i+1, col]):
                    # do nothing
                    pass
                else:
                    for j in range(i+1, q_as.shape[0]):
                        if q_as[i, col] < q_as[j, col]:
                            idx = j
                            break
                        elif j == q_as.shape[0]-1:
                            idx = q_as.shape[0]

                    q_as_part = self.sort_joints(q_as[i:idx, :], col+1)
                    q_as[i:idx, :] = q_as_part
        else:
            q_as = q[q[:, -1].argsort()]
        return q_as