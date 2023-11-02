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
        
        q1 = joints_123[0][0]
        q2 = joints_123[0][1]
        q3 = joints_123[0][2]
        q4 = joints_467[0][0]
        q5 = 0
        q6 = joints_467[0][1]
        q7 = joints_467[0][2]

        q = [[q1,q2,q3,q4,q5,q6,q7]]
         
        #use the joint limits in order to get rid of the inputs 
        #failint joint 
        self.lowerLim = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]).reshape((1, 7))    # Lower joint limits in radians ** This does not include gripper
        self.upperLim = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973]).reshape((1, 7))

        #for i in range (0,7):
        #    if joints_123[i][0] > -2.8973 and joints_123[i][0] < 2.8973 
       
        # Student's code goes in between:
        
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

        t_07 = target["t"]
        print("------t-07-------\n",t_07)
        R_transpose = np.transpose(R_07)
        
        o_07 = t_07
        print("-------o_07-----\n",o_07)
        
        o_27 = o_07+(self.d1 * np.matmul(R_transpose, [[0],[0],[1]]))
        print("-----o_27------\n",o_27)
        wrist_pos = o_27
        print("-----wrist_pos------\n",wrist_pos)
       
    
        return wrist_pos

    def ik_pos(self, wrist_pos):
        """
        Solves IK position problem on the joint 4, 6, 7
        Args:
            wrist_pos: 3x1 numpy array of the position of the wrist center in frame 7

        Returns:
             joints_467 = nx3 numpy array of all joint angles of joint 4, 6, 7

        """
        #if neg add pi if pos sub pi
        q7 = (5*pi)/4 - (math.atan2(wrist_pos[1,0],wrist_pos[0,0]))
        if wrist_pos[0,0] == 0 and wrist_pos[1,0] == 0: 
            q7 = self.Q0
        if q7 >= 0: 
            q7_flip = q7 - pi
        else: 
            q7_flip = q7 + pi
        
        T_67 = self.createFullMatrix(0,0,self.d7,q7 - pi/4)
        print("------T_67----\n",T_67)
        T_67_flip = self.createFullMatrix(0,0,self.d7,q7_flip - pi/4)
        
        
        o_27 = np.array([wrist_pos[0],wrist_pos[1],wrist_pos[2],[1]])
        print("--------o_27,IK posiition-----\n",o_27)
        
        

        o_62 = np.dot(T_67,o_27)
        o_62_flip = np.matmul(T_67_flip,o_27)


        print("-------o_62-----\n", o_62)
        o_25_6 = np.array([[o_62[0,0]+self.a6] ,[o_62[1,0]],[o_62[2,0]]]) #+ np.array([[self.a6],[0],[0]])
        o_25_6_flip = np.array([[o_62_flip[0][0] + self.a6],[o_62_flip[1][0]],[o_62_flip[2][0]]])
        
        a = np.sqrt(self.d5**2+self.a3**2)
        b = np.sqrt(self.d3**2+self.a5**2)

        cos_2up =  (o_25_6[0]**2 + o_25_6[2]**2 - a**2 - b**2) / (2*a*b)
        cos_2up = np.clip(cos_2up,-1,1)
        theta2_up = np.arccos(cos_2up)
    
        cos_flip2up = (o_25_6_flip[0]**2 + o_25_6_flip[2]**2 - a**2 - b**2) / (2*a*b)
        cos_flip2up = np.clip(cos_flip2up,-1,1)
        theta2_flip_up = np.arccos(cos_flip2up)

        cos_2down = -(o_25_6[0]**2 + o_25_6[2]**2 - a**2 - b**2) / (2*a*b)
        cos_2down = np.clip(cos_2down,-1,1)
        theta2_down = np.arccos(cos_2down)

        cos_flip2down = -(o_25_6_flip[0]**2 + o_25_6_flip[2]**2 - a**2 - b**2) / (2*a*b)
        cos_flip2down = np.clip(cos_flip2down,-1,1)
        theta2_flip_down = np.arccos(-(o_25_6_flip[0]**2 + o_25_6_flip[2]**2 - a**2 - b**2) / (2*a*b))

        theta1_up = math.atan2(o_25_6[0],o_25_6[2]) - math.atan2((b*np.sin(theta2_up)),(a+b*np.cos(theta2_up)))
        theta1_flip_up = math.atan2(o_25_6_flip[0],o_25_6_flip[2]) - math.atan2((b*np.sin(theta2_flip_up)),(a+b*np.cos(theta2_flip_up)))

        theta1_down = math.atan2(o_25_6[0],o_25_6[2]) + math.atan2((b*np.sin(theta2_down)),(a+b*np.cos(theta2_down)))
        theta1_flip_down = math.atan2(o_25_6_flip[0],o_25_6_flip[2]) + math.atan2((b*np.sin(theta2_flip_down)),(a+b*np.cos(theta2_flip_down)))

        #create versions for elbow up and elbow down 
        #should get positve and negative q7 and q6
        q4_up = theta2_up + math.atan2(self.d3,self.a3) + math.atan2(self.d5,self.a3) - pi
        q4_flip_up = theta2_flip_up + math.atan2(self.d3,self.a3)+math.atan2(self.d5,self.a3) - pi
        q4_down = theta2_down + math.atan2(self.d3,self.a3)+math.atan2(self.d5,self.a3) - pi
        q4_flip_down = theta2_flip_down + math.atan2(self.d3,self.a3)+math.atan2(self.d5,self.a3) - pi
        

        q6_up = theta1_up - pi/2 + math.atan2(self.a3,self.a5)
        q6_flip_up = theta1_flip_up - pi/2 + math.atan2(self.a3,self.a5)
        q6_down = theta1_down - pi/2 + math.atan2(self.a3,self.a5)
        q6_flip_down = theta1_flip_down - pi/2 + math.atan2(self.a3,self.a5)
        
        print("-----TEST----\n",[q4_up,q6_up,q7])


        joints_467 =  [[q4_up,q6_up,q7],
                                [q4_flip_up, q6_flip_up, q7_flip],
                                [q4_down,q6_down,q7],
                                [q4_flip_down,q6_flip_down,q7_flip]]

        print("---------JOINTS_467-------- \n",joints_467)
        return joints_467

    def ik_orient(self, R, joints_467):
        state = True
        """
        Solves IK orientation problem on the joint 1, 2, 3
        Args:
            R: numpy array of the end effector pose relative to the robot base
            joints_467: nx3 numpy array of all joint angles of joint 4, 6, 7

        Returns:
            joints_123 = nx3 numpy array of all joint angles of joint 1, 2 ,3 (a 4x3 right now)
        """
       
        #make the rotation matrices
        for i in range(0,2): 
            q4 = joints_467[i][0]
            
            q6 = joints_467[i][1]
            q7 = joints_467[i][2]
        
            A4 = self.createRotMatrix(-pi/2, q4)
            print("---A4-----\n",A4)
            A5 = self.createRotMatrix(pi/2,0)
            A6 = self.createRotMatrix(pi/2,q6)
            A7 = self.createRotMatrix(0,q7-pi/4)

            R_37 = A4@A5@A6@A7

            R_37_transpose = np.transpose(R_37)
            print("---R_37----\n",R_37)
            R_03 = np.array(np.matmul(R_37, np.transpose(R)))
            #print("----R_03---\n",R_03)
            print("----R_03---\n",R_03[0][0])
    
        #q1 = phi, q2 = theta q3 = psi
            
            q1 = math.atan2(-R_03[2][1], R_03[2][0])
            q2 = np.arccos(R_03[2][2])
            q3 = math.atan2(R_03[1][2],R_03[0][2])
           
            joints_123 = np.array([[q1,q2,q3]])
            joints_123 = np.vstack((joints_123, [q1-pi, -q2, q3-pi]))
            joints_123 = np.vstack((joints_123, [q1, q2, q3]))
            joints_123 = np.vstack((joints_123, [q1-pi, -q2, q3-pi]))


          
        print("-----R_37----\n",R_37)
        #print(joints_123)
        print("----joints_123----\n",joints_123)
        return joints_123

    def createRotMatrix(self, alpha, theta): 
        A = [[np.cos(theta), -1*np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha)],
                     [np.sin(theta), np.cos(theta)*np.cos(alpha), -1*np.cos(theta)*np.sin(alpha)],
                     [0, np.sin(alpha), np.cos(alpha)]]
        return A

    def createFullMatrix(self,a,alpha,d,theta):
        T = [[np.cos(theta), -1*np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                        [np.sin(theta), np.cos(theta)*np.cos(alpha), -1*np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                        [0, np.sin(alpha), np.cos(alpha), d],
                        [0, 0, 0, 1]]
        return T

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
    #should ideally be 0 

    for i in range(q.shape[0]):
        [_, T_ik] = fk.forward(q[i, :])
        print('Matrix difference = ')
        print(T_fk - T_ik)
        print()

if __name__ == '__main__':
    main()
