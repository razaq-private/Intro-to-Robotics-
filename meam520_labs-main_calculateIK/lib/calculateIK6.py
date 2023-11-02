import numpy as np
import math
from math import pi
from calculateFK import FK



#call with self.forward
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
    Q0 = 123.45


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
        wrist_pos = self.kin_decouple(target)
        joints_467 = self.ik_pos(wrist_pos)
        joints_123 = self.ik_orient(target["R"],joints_467)
        
        q1 = []
        q2 = []
        q3 = []
        q4 = []
        q5 = []
        q6 = []
        q7 = []

        #q1.append(joints_123[1][0])
        for i in range(0,7): 
            q1.append(joints_123[i][0])
            q2.append(joints_123[i][1])
            q3.append(joints_123[i][2])
            q4.append(joints_467[i][0])
            q5.append(0)
            q6.append(joints_467[i][1])
            q7.append(joints_467[i][2])
        
        q_row1 = [q1[0],q2[0],q3[0],q4[0],q5[0],q6[0],q7[0]]
        q_row2 = [q1[1],q2[1],q3[1],q4[1],q5[1],q6[1],q7[1]]
        q_row3 = [q1[2],q2[2],q3[2],q4[2],q5[2],q6[2],q7[2]]
        q_row4 = [q1[3],q2[3],q3[3],q4[3],q5[3],q6[3],q7[3]]
        q_row5 = [q1[4],q2[4],q3[4],q4[4],q5[4],q6[4],q7[4]]
        q_row6 = [q1[5],q2[5],q3[5],q4[5],q5[5],q6[5],q7[5]]
        q_row7 = [q1[6],q2[6],q3[6],q4[6],q5[6],q6[6],q7[6]]
        q = [q_row1,q_row2,q_row3,q_row4,q_row5,q_row6,q_row7]
        q_new = []
        print("----q---\n",q[0][0])

        
        
        print("----q1_row--\n",q_row2)

        self.lowerLim = [-2.8973, -1.7628, -2.8973, -3.0718, -0.0175, -2.8973]   # Lower joint limits in radians ** This does not include gripper
        self.upperLim = [ 2.8973,  1.7628,  2.8973, -0.0698,  3.7525,  2.8973]          # Upper joint limits in radians (grip in mm)
        
        print("---q_upper---\n", self.upperLim[0])
       #
       #
       #
       #
       #
       #failing at joint 6 
        for i in range(0,7): 
            
            if q[i][0] < self.upperLim[0] and q[i][0] > self.lowerLim[0] and q[i][1] < self.upperLim[1] and q[i][1] > self.lowerLim[1] and  q[i][2] < self.upperLim[2] and q[i][2] > self.lowerLim[2] and q[i][3] < self.upperLim[3] and q[i][3] > self.lowerLim[3] and q[i][5] < self.upperLim[4] and q[i][5] > self.lowerLim[4] and q[i][6] < self.upperLim[5] and q[i][6] > self.lowerLim[5]:
                q_new.append(q[i]) 
               
         #q = np.array([[joints_123[0][0], joints_123[0][1], joints_123[0][2], joints_467[0][0], 0, joints_467[0][1], joints_467[0][2]]])

        print("-----new_q FINAL----\n", q_new)
        q = np.array([q_new])
        #q = np.array([[1,1,1,1,0,1,1]])
            #Student's code goes in between:
        
        ## DO NOT EDIT THIS PART
        # This will convert your joints output to the autograder format
        q = self.sort_joints(q)
        ## DO NOT EDIT THIS PART
        return q

    def kin_decouple(self, target):
        """
        Performs kinematic decoupling on the panda arm to find the position of wrist center
        Args:
            target: dictionary containing:
                'R': numpy array of the end effector pose relative to the robot base
                't': numpy array of the end effector position relative to the robot base

        Returns:
             wrist_pos = 3x1 numpy array of the position of the wrist center in frame 7
        """
        
        R_07 = target["R"]
       
        t_07 = target["t"].reshape(3,1)
        R_transpose = np.transpose(R_07)

        o_07 = -1*np.dot(R_transpose, t_07)

        o_27 = self.d1 * np.matmul(R_transpose, [[0],[0],[1]],dtype=object,) + o_07


        wrist_pos = o_27
        
        return wrist_pos

    def ik_pos(self, wrist_pos):
        """
        Solves IK position problem on the joint 4, 6, 7
        Args:
            wrist_pos: 3x1 numpy array of the position of the wrist center in frame 7

        Returns:
             joints_467 = nx3 numpy array of all joint angles of joint 4, 6, 7

        """

        #q7 = pi - (math.atan2(wrist_pos[1],wrist_pos[0])+pi/4)
        q7 = 5*pi/4 - math.atan2(wrist_pos[1],wrist_pos[0])
        if wrist_pos[0][0] == 0 and wrist_pos[1][0] == 0: 
            q7 = self.Q0
     
            
        if q7 <= -2.8973:
            q7 = q7+pi
        if q7 >= -2.8973:
            q7 = q7-pi

        
        T_67_values = [0, 0, self.d7, q7-pi/4]
        T_67 = np.array([[np.cos(T_67_values[3]), -1*np.sin(T_67_values[3])*np.cos(T_67_values[2]), np.sin(T_67_values[3])*np.sin(T_67_values[2]), 0],
                        [np.sin(T_67_values[3]), np.cos(T_67_values[3])*np.cos(0), -1*np.cos(T_67_values[3])*np.sin(0), 0*np.sin(0)],
                        [0, np.sin(0), np.cos(0), T_67_values[2]],
                        [0, 0, 0, 1]])
        
        
        o_27 = np.array([wrist_pos[0][0],wrist_pos[1][0], wrist_pos[2][0],[1]],dtype=object,)
        print("-----o_27----\n",o_27)
        o_62 = np.matmul(T_67,o_27)
        o_25_6 = [o_62[0][0] + self.a6,o_62[1][0],o_62[2][0]]
        

        a = np.sqrt(self.d5**2+self.a3**2)
        b = np.sqrt(self.d3**2+self.a5**2)

        #theta for one solution
        theta2 = np.arccos((o_25_6[0]**2 + o_25_6[2]**2 - a**2 - b**2) / (2*a*b))
        theta1 = math.atan2(o_25_6[2],o_25_6[0]) - math.atan2(b*np.sin(theta2),a+b*np.cos(theta2))
        print("---theta2----\n", theta2)

        q4 = theta2 - pi + math.atan2(self.d3,self.a3) + math.atan2(self.d5,self.a3)
        q6 = theta1 + pi/2 + np.arctan(self.a3/self.d5)

        #theta for other set of solutions
        theta2_1 = -np.arccos((o_25_6[0]**2 + o_25_6[2]**2 - a**2 - b**2) / (2*a*b))
        theta1_1 = math.atan2(o_25_6[2],o_25_6[0]) - math.atan2(b*np.sin(theta2),a+b*np.cos(theta2)) 

        q4_1 = theta2_1 - pi + math.atan2(self.d3,self.a3)+math.atan2(self.d5,self.a3)
        q6_1 = theta1_1 - pi/2 + np.arctan(self.a3/self.d5)

        theta2_2 = np.arccos((o_25_6[0]**2 + o_25_6[2]**2 - a**2 - b**2) / (2*a*b))
        theta1_2 = -(math.atan2(o_25_6[2],o_25_6[0]) - math.atan2(b*np.sin(theta2),a+b*np.cos(theta2)))
        q4_2 = theta2_2 - pi + math.atan2(self.d3,self.a3)+math.atan2(self.d5,self.a3)
        q6_2 = theta1_2 - pi/2 + np.arctan(self.a3/self.d5)

        theta2_3 = -np.arccos((o_25_6[0]**2 + o_25_6[2]**2 - a**2 - b**2) / (2*a*b))
        theta1_3 = -(math.atan2(o_25_6[2],o_25_6[0]) - math.atan2(b*np.sin(theta2),a+b*np.cos(theta2)))
        q4_3 = theta2_3 - pi + math.atan2(self.d3,self.a3)+math.atan2(self.d5,self.a3)
        q6_3 = theta1_3 - pi/2 + np.arctan(self.a3/self.d5)

        joints_467 = [[q4,q6,q7],[q4,q6,q7],[q4_1,q6_1,q7],[q4_1,q6_1,q7],[q4_2,q6_2,q7],[q4_2,q6_2,q7],[q4_3,q6_3,q7],[q4_3,q6_3,q7]]

        print("-----joints 467-----\n", joints_467)
        return joints_467

    def ik_orient(self, R, joints_467):
        """
        Solves IK orientation problem on the joint 1, 2, 3
        Args:
            R: numpy array of the end effector pose relative to the robot base
            joints_467: nx3 numpy array of all joint angles of joint 4, 6, 7

        Returns:
            joints_123 = nx3 numpy array of all joint angles of joint 1, 2 ,3
        """
        joints_123 = []
        for i in range(0,len(joints_467),2):
            Anew = [[1,0,0],[0,0,-1],[0,1,0]]
            A4 = [[np.cos(joints_467[i][0]), -1*np.sin(joints_467[i][0])*np.cos(-pi/2), np.sin(joints_467[i][0])*np.sin(-pi/2)],
                [np.sin(joints_467[i][0]), np.cos(joints_467[i][0])*np.cos(-pi/2), -1*np.cos(joints_467[i][0])*np.sin(-pi/2)],
                [0, np.sin(-pi/2), np.cos(-pi/2)]]
            A5 = [[np.cos(0), -1*np.sin(0)*np.cos(pi/2), np.sin(0)*np.sin(pi/2)],
                [np.sin(0), np.cos(0)*np.cos(pi/2), -1*np.cos(0)*np.sin(pi/2)],
                [0, np.sin(pi/2), np.cos(pi/2)]]
            A6 = [[np.cos(joints_467[i][1]), -1*np.sin(joints_467[i][1])*np.cos(pi/2), np.sin(joints_467[i][1])*np.sin(pi/2)],
                [np.sin(joints_467[i][1]), np.cos(joints_467[i][1])*np.cos(pi/2), -1*np.cos(joints_467[i][1])*np.sin(pi/2)],
                [0, np.sin(pi/2), np.cos(pi/2)]]
            A7 = [[np.cos(joints_467[i][2]-pi/4), -1*np.sin(joints_467[i][2]-pi/4)*np.cos(0), np.sin(joints_467[i][2]-pi/4)*np.sin(0)],
                [np.sin(joints_467[i][2]-pi/4), np.cos(joints_467[i][2]-pi/4)*np.cos(0), -1*np.cos(joints_467[i][2]-pi/4)*np.sin(0)],
                [0, np.sin(0), np.cos(0)]]
            R_37 = np.matmul(Anew,np.matmul(A4,np.matmul(A5,np.matmul(A6,A7))))
            R_37_transpose = np.transpose(R_37)
            R_03 = np.matmul(R,R_37_transpose)
            
            q2 = np.arccos(np.round(R_03[2,2],7))
            #switch between the q's 
            
            q2 = q2
           
            if q2 == 0: 
                q1 = self.Q0
                q3 = math.atan2(R_03[2,1]/np.sin(q2),R_03[2,0]/-np.sin(q2))
                joints_123.append([q1,q2,q3])
            else: 
                if joints_467[i][2] == self.Q0:
                    q1 = self.Q0
                    q3 = math.atan2(R_03[2,1]/np.sin(q2),R_03[2,0]/-np.sin(q2))
                    joints_123.append([q1,q2,q3])
                else: 
                    q1 = math.atan2(R_03[1,2]/np.sin(q2),R_03[0,2]/-np.sin(q2))
                    q3 = math.atan2(R_03[2,1]/np.sin(q2),R_03[2,0]/-np.sin(q2))
                    joints_123.append([q1,q2,q3])

        for i in range(1,len(joints_467),2):
            Anew = [[1,0,0],[0,0,-1],[0,1,0]]
            A4 = [[np.cos(joints_467[i][0]), -1*np.sin(joints_467[i][0])*np.cos(-pi/2), np.sin(joints_467[i][0])*np.sin(-pi/2)],
                [np.sin(joints_467[i][0]), np.cos(joints_467[i][0])*np.cos(-pi/2), -1*np.cos(joints_467[i][0])*np.sin(-pi/2)],
                [0, np.sin(-pi/2), np.cos(-pi/2)]]
            A5 = [[np.cos(0), -1*np.sin(0)*np.cos(pi/2), np.sin(0)*np.sin(pi/2)],
                [np.sin(0), np.cos(0)*np.cos(pi/2), -1*np.cos(0)*np.sin(pi/2)],
                [0, np.sin(pi/2), np.cos(pi/2)]]
            A6 = [[np.cos(joints_467[i][1]), -1*np.sin(joints_467[i][1])*np.cos(pi/2), np.sin(joints_467[i][1])*np.sin(pi/2)],
                [np.sin(joints_467[i][1]), np.cos(joints_467[i][1])*np.cos(pi/2), -1*np.cos(joints_467[i][1])*np.sin(pi/2)],
                [0, np.sin(pi/2), np.cos(pi/2)]]
            A7 = [[np.cos(joints_467[i][2]-pi/4), -1*np.sin(joints_467[i][2]-pi/4)*np.cos(0), np.sin(joints_467[i][2]-pi/4)*np.sin(0)],
                [np.sin(joints_467[i][2]-pi/4), np.cos(joints_467[i][2]-pi/4)*np.cos(0), -1*np.cos(joints_467[i][2]-pi/4)*np.sin(0)],
                [0, np.sin(0), np.cos(0)]]
            R_37 = np.matmul(Anew,np.matmul(A4,np.matmul(A5,np.matmul(A6,A7))))
            R_37_transpose = np.transpose(R_37)
            R_03 = np.matmul(R,R_37_transpose)
            
            q2 = np.arccos(np.round(R_03[2,2],7))
            #switch between the q's 
            
            q2 = -q2
                
            if q2 == 0: 
                q1 = self.Q0
                q3 = math.atan2(R_03[2,1]/np.sin(q2),R_03[2,0]/-np.sin(q2))
                joints_123.append([q1,q2,q3])
            else: 
                if joints_467[i][2] == self.Q0:
                    q1 = self.Q0
                    q3 = math.atan2(R_03[2,1]/np.sin(q2),R_03[2,0]/-np.sin(q2))
                    joints_123.append([q1,q2,q3])
                else: 
                    q1 = math.atan2(R_03[1,2]/np.sin(q2),R_03[0,2]/-np.sin(q2))
                    q3 = math.atan2(R_03[2,1]/np.sin(q2),R_03[2,0]/-np.sin(q2))
                    joints_123.append([q1,q2,q3])
      

        
        
        return joints_123

    
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

def main():

    # fk solution code
    fk = FK()

    # input joints
    q1 = 0
    q2 = 0
    q3 = 0
    q4 = -np.pi/2
    q6 = 0
    q7 = 0

    q_in  = np.array([q1, q2, q3, q4, 0, q6, q7])
    [_, T_fk] = fk.forward(q_in)

    # input of IK class
    target = {'R': T_fk[0:3, 0:3], 't': T_fk[0:3, 3]}
    ik = IK()
    q = ik.panda_ik(target)

    # verify IK solutions
    for i in range(q.shape[0]):
        [_, T_ik] = fk.forward(q[i, :])
        print('Matrix difference = ')
        print(T_fk - T_ik)
        print()

if __name__ == '__main__':
    main()