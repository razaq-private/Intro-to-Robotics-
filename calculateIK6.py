import numpy as np
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
        R_transpose = np.transpose(R_07)
        
        o_07 = np.dot(-1*R_transpose, t_07)

        o_27 = self.d1 * np.matmul(R_transpose, [[0],[0],[1]]) + o_07

        #T_70 = np.matrix([R_transpose[0][0], R_transpose[0][1], R_transpose[0][2], o_07[0]],
            #            [R_transpose[1][0], R_transpose[1][1], R_transpose[1][2], o_07[1]],
             #           [R_transpose[2][0], R_transpose[2][1], R_transpose[2][2], o_07[2]],
            #            [0,0,0,1])
        
        

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
        T_67 = np.matrix(0,0,self.d7,q7)
        
        a = np.sqrt(self.d5^2+self.a3^2)
        b = np.sqrt(self.d3^2+self.a5^2)
        theta2 = (wrist_pos[0]^2 + wrist_pos[1]^2 - a^2 - b^2)/(2*a*b)
        theta1 = np.atan2(wrist_pos[0]/wrist_pos[1]) - np.atan2((b*np.sin(theta2))/a+b*np.cos(theta2)) 

        q4 = theta2 + np.atan2(self.d3/self.a3)+np.atan2(self.d5/self.a3) 
        q6 = theta1 - pi/2 + np.atan2(self.a3/self.a5)
        q7 = pi - (np.atan2(wrist_pos[1]/wrist_pos[0])+pi/4)

        joints_467 = [q4,q6,q7]
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
        R_03 = np.matrix([])
        psi = []
        theta = []
        phi = [] 

        joints_123 = [] 
        return joint_123
    
    def sort_joints(q, col=0):
        """
        Sort the joint angle matrix by ascending order 
        Args: 
            q: nx7 joint angle matrix 
        Returns: 
            q_as = nx7 joint angle matrix in ascending order 
        """
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

                q_as_part = sort_joints(q_as[i:idx, :], col+1)
                q_as[i:idx, :] = q_as_part
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












