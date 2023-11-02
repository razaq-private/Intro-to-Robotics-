import numpy as np
from math import pi, acos
import math
from scipy.linalg import null_space
from copy import deepcopy
'''
from lib.calcJacobian import calcJacobian
from lib.calculateFK import FK
from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
'''
from calcJacobian import calcJacobian
from calculateFK import FK
from detectCollision import detectCollision
from loadmap import loadmap


class PotentialFieldPlanner:

    # JOINT LIMITS
    lower = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upper = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    center = lower + (upper - lower) / 2 # compute middle of range of motion of each joint
    fk = FK()

    def __init__(self, tol=1e-4, max_steps=500, min_step_size=1e-5):
        """
        Constructs a potential field planner with solver parameters.

        PARAMETERS:
        tol - the maximum distance between two joint sets
        max_steps - number of iterations before the algorithm must terminate
        min_step_size - the minimum step size before concluding that the
        optimizer has converged
        """

        # YOU MAY NEED TO CHANGE THESE PARAMETERS

        # solver parameters
        self.tol = tol
        self.max_steps = max_steps
        self.min_step_size = min_step_size


    ######################
    ## Helper Functions ##
    ######################
    # The following functions are provided to you to help you to better structure your code
    # You don't necessarily have to use them. You can also edit them to fit your own situation 

    @staticmethod
    def attractive_force(target, current):
        """
        Helper function for computing the attactive force between the current position and
        the target position for one joint. Computes the attractive force vector between the 
        target joint position and the current joint position 

        INPUTS:
        target - 3x1 numpy array representing the desired joint position in the world frame
        current - 3x1 numpy array representing the current joint position in the world frame

        OUTPUTS:
        att_f - 3x1 numpy array representing the force vector that pulls the joint 
        from the current position to the target position 
        """

        ## STUDENT CODE STARTS HERE

        #attractive force is the difference between the desired location and the location of the joints 

        

        att_f = np.zeros((3, 1)) 
        print('---att force current---\n',current)
        print(target)
        att_f = current - target

        ## END STUDENT CODE

        return att_f

    @staticmethod
    def repulsive_force(obstacle, current, unitvec=np.zeros((3,1))):
        """
        Helper function for computing the repulsive force between the current position
        of one joint and one obstacle. Computes the repulsive force vector between the 
        obstacle and the current joint position 

        INPUTS:
        obstacle - 1x6 numpy array representing the an obstacle box in the world frame
        current - 3x1 numpy array representing the current joint position in the world frame
        unitvec - 3x1 numpy array representing the unit vector from the current joint position 
        to the closest point on the obstacle box 

        OUTPUTS:
        rep_f - 3x1 numpy array representing the force vector that pushes the joint 
        from the obstacle
        """

        ## STUDENT CODE STARTS HERE

        rep_f = np.zeros((3, 1)) 

        #field strength manipulator 
        #need to find the distances of the different vectors 
        #distance between the object and obstacle & distance of the boundary of influence 
        p_oiq, unitvec = PotentialFieldPlanner.dist_point2box(np.transpose(current), obstacle)
        p_oiq = p_oiq * 0.5
        #unitvec = -np.reshape(unitvec, (3,1))

        distanceOfInfluence = 1
        ni = 1
        

        rep_f = ni * ((1/p_oiq) - (1/distanceOfInfluence)) * (1/p_oiq**2) * unitvec


        ## END STUDENT CODE

        return rep_f 



    @staticmethod
    def dist_point2box(p, box):
        """
        Helper function for the computation of repulsive forces. Computes the closest point
        on the box to a given point

        INPUTS:
        p - nx3 numpy array of points [x,y,z]
        box - 1x6 numpy array of minimum and maximum points of box

        OUTPUTS:
        dist - nx1 numpy array of distance between the points and the box
                dist > 0 point outside
                dist = 0 point is on or inside box
        unit - nx3 numpy array where each row is the corresponding unit vector
        from the point to the closest spot on the box
            norm(unit) = 1 point is outside the box
            norm(unit)= 0 point is on/inside the box

         Method from MultiRRomero
         @ https://stackoverflow.com/questions/5254838/
         calculating-distance-between-a-point-and-a-rectangular-box-nearest-point
        """
        # THIS FUNCTION HAS BEEN FULLY IMPLEMENTED FOR YOU

        # Get box info
        boxMin = np.array([box[0], box[1], box[2]])
        boxMax = np.array([box[3], box[4], box[5]])
        boxCenter = boxMin*0.5 + boxMax*0.5
        p = np.array(p)

        # Get distance info from point to box boundary
        dx = np.amax(np.vstack([boxMin[0] - p[:, 0], p[:, 0] - boxMax[0], np.zeros(p[:, 0].shape)]).T, 1)
        dy = np.amax(np.vstack([boxMin[1] - p[:, 1], p[:, 1] - boxMax[1], np.zeros(p[:, 1].shape)]).T, 1)
        dz = np.amax(np.vstack([boxMin[2] - p[:, 2], p[:, 2] - boxMax[2], np.zeros(p[:, 2].shape)]).T, 1)



        # convert to distance
        distances = np.vstack([dx, dy, dz]).T
        dist = np.linalg.norm(distances, axis=1)

        # Figure out the signs
        signs = np.sign(boxCenter-p)

        # Calculate unit vector and replace with
        unit = distances / dist[:, np.newaxis] * signs
        unit[np.isnan(unit)] = 0
        unit[np.isinf(unit)] = 0
        return dist, unit

    @staticmethod
    def compute_forces(target, obstacle, current):
        """
        Helper function for the computation of forces on every joints. Computes the sum 
        of forces (attactive, repulsive) on each joint. 

        INPUTS:
        target - 3x7 numpy array representing the desired joint/end effector positions 
        in the world frame
        obstacle - nx6 numpy array representing the obstacle box min and max positions
        in the world frame
        current- 3x7 numpy array representing the current joint/end effector positions 
        in the world frame

        OUTPUTS:
        joint_forces - 3x7 numpy array representing the force vectors on each 
        joint/end effector
        """
        print("----current----\n",current)
        current = np.transpose(current)
        target = np.transpose(target)

        ## STUDENT CODE STARTS HERE

        #rep_f = ni * ((1/p_oiq) - (1/distanceOfInfluence)) * (1/p_oiq^2) * unitvec

        f_att = np.zeros((3,7))

        #get attractive force 
        
        f_att[:,0] = PotentialFieldPlanner.attractive_force(target[:,0], current[:,0])

       
        #get repulsive force 

        unitvec = np.zeros((1,7)) + 1 
        f_repFinal = np.zeros((3,7))
        f_rep = np.zeros((3,7))
        for i in range(len(obstacle)): 
            for j in range(0,7): 
                currentInput = current[:,j]
                f_rep[:,j] = PotentialFieldPlanner.repulsive_force(obstacle[i], currentInput, unitvec)

            f_repFinal += f_rep
        

    
        
        joint_forces = f_att + f_repFinal
 
        #joint_forces = np.reshape((f_att+f_repFinal), (3,7))


        
        ## END STUDENT CODE

        return joint_forces
    
    @staticmethod
    def compute_torques(joint_forces, q):
        """
        Helper function for converting joint forces to joint torques. Computes the sum 
        of torques on each joint.

        INPUTS:
        joint_forces - 3x7 numpy array representing the force vectors on each 
        joint/end effector
        q - 1x7 numpy array representing the current joint angles

        OUTPUTS:
        joint_torques - 1x7 numpy array representing the torques on each joint 
        """

        ## STUDENT CODE STARTS HERE

        #torque is the jacboians * the force

        #take the torques of the individual forces and then sum them up 

        """
        When a force acts on a point on the robot, any joints chained between that point and the base will need
        to generate some torque to balance the force. In general, you should input a 3x1 force and a 3x7 Jacobian
        for each joint, giving you a 7x1 torque for all your joints. So your second proposal is on the right track.
        Thought experiment: if you apply a force on the "i" joint, what torque should the "i+1" joint contribute to
        maintaining force balance? Recall that the Jacobian is different when you apply forces on other joints.
        Once you calculate the torques resulting from all the forces applied to each joint, you can sum it up, joint
        by joint.
        """

        #set the joints after a certain point to equal 0 for instance for joint 1 only the jacobian for 1 should have a value 

    
        J = calcJacobian(q)
        Jv = J[0:3,:]
        Jv1 = np.hstack((Jv[:,0:1], np.zeros((3,6))))
        Jv2 = np.hstack((Jv[:,0:2], np.zeros((3,5))))
        Jv3 = np.hstack((Jv[:,0:3], np.zeros((3,4))))
        Jv4 = np.hstack((Jv[:,0:4], np.zeros((3,3))))
        Jv5 = np.hstack((Jv[:,0:5], np.zeros((3,1))))
        Jv6 = np.hstack((Jv[:,0:6], np.zeros((3,1))))
        Jv7 = Jv 

        torque1 = np.transpose(Jv1) @ np.reshape(joint_forces[:,0], (3,1))
        torque2 = np.transpose(Jv2) @ np.reshape(joint_forces[:,1], (3,1))
        torque3 = np.transpose(Jv3) @ np.reshape(joint_forces[:,2], (3,1))
        torque4 = np.transpose(Jv4) @ np.reshape(joint_forces[:,3], (3,1))
        torque5 = np.transpose(Jv5) @ np.reshape(joint_forces[:,4], (3,1))
        torque6 = np.transpose(Jv6) @ np.reshape(joint_forces[:,5], (3,1))
        torque7 = np.transpose(Jv7) @ np.reshape(joint_forces[:,6], (3,1))
        '''
        J = calcJacobian(q)
        joint_torques = np.zeros((1,7))

        for i in range(7): 
            if i != 6: 
                J_joints = J[i]
                J_joints[:,i+1:7] =  np.zeros((3,(7-1-i)))
            else: 
                J_joints = J[6]
            torque = np.transpose(J_joints) @ joint_forces[:,i]
            joint_torques += torque
        '''
        


       

        #joint_torques = np.zeros((1, 7)) 

        ## END STUDENT CODE

        return joint_torques

    @staticmethod
    def q_distance(target, current):
        """
        Helper function which computes the distance between any two
        vectors.

        This data can be used to decide whether two joint sets can be
        considered equal within a certain tolerance.

        INPUTS:
        target - 1x7 numpy array representing some joint angles
        current - 1x7 numpy array representing some joint angles

        OUTPUTS:
        distance - the distance between the target and the current joint sets 

        """

        ## STUDENT CODE STARTS HERE
        '''
        distance = math.sqrt(abs((target[0]**2 - current[0]**2) + (target[1]**2 - current[1]**2) + (target[2]**2 - current[2]**2) + (target[3]**2 - current[3]**2) + (target[4]**2 - current[4]**2) + (target[5]**2 - current[5]**2) + (target[6]**2 - current[6]**2)))
        '''
        distance = np.linalg.norm(target - current)


        ## END STUDENT CODE

        return distance
    
    @staticmethod
    def compute_gradient(q, target, map_struct):
        """
        Computes the joint gradient step to move the current joint positions to the
        next set of joint positions which leads to a closer configuration to the goal 
        configuration 
        
        INPUTS:
        q - 1x7 numpy array. the current joint configuration, a "best guess" so far for the final answer
        target - 1x7 numpy array containing the desired joint angles
        map_struct - a map struct containing the obstacle box min and max positions

        OUTPUTS:
        dq - 1x7 numpy array. a desired joint velocity to perform this task
        """

        #convert from task space to the configuration space done 

        ## STUDENT CODE STARTS HERE

        dq = np.zeros((1, 7))

        stepSize = 1
        obstacles = map_struct.obstacles

        current, Aoe = np.transpose(PotentialFieldPlanner.fk.forward(q))
        targetPos, AoeTarget = np.transpose(PotentialFieldPlanner.fk.forward(target))

        joint_forces = PotentialFieldPlanner.compute_forces(targetPos[:,:-1], map_struct.obstacles, current[:,:-1])
        joint_torques = PotentialFieldPlanner.compute_torques(joint_forces, q)

        normalizedJointTorques = joint_torques/np.linalg.norm(joint_torques)

        dq = stepSize * normalizedJointTorques

 
        ## END STUDENT CODE

        return dq

    ###############################
    ### Potential Feild Solver  ###
    ###############################

    def plan(self, map_struct, start, goal):
        """
        Uses potential field to move the Panda robot arm from the startng configuration to
        the goal configuration.

        INPUTS:
        map_struct - a map struct containing min and max positions of obstacle boxes 
        start - 1x7 numpy array representing the starting joint angles for a configuration 
        goal - 1x7 numpy array representing the desired joint angles for a configuration

        OUTPUTS:
        q - nx7 numpy array of joint angles [q0, q1, q2, q3, q4, q5, q6]. This should contain
        all the joint angles throughout the path of the planner. The first row of q should be
        the starting joint angles and the last row of q should be the goal joint angles. 
        """
        q = start
        q_path = np.array([]).reshape(0,7)
        q_path = np.reshape(np.append(q_path, start), (1,7))
        
        startPosition, Aoe = PotentialFieldPlanner.fk.forward(start)
        goalPosition, Aee = PotentialFieldPlanner.fk.forward(goal)
        endEffectorPos = np.array([])
        endEffectorPos = np.reshape(np.append(endEffectorPos, startPosition[7,:]), (1,3))


        count = 0

        while True:

            ## STUDENT CODE STARTS HERE
            
            # The following comments are hints to help you to implement the planner
            # You don't necessarily have to follow these steps to complete your code 
            
            # Compute gradient
            a = 1
            gradient = PotentialFieldPlanner.compute_gradient(q, goal, map_struct)
            q = q + gradient * a 

            #check the error 
            checkError, x =  PotentialFieldPlanner.fk.forward(q_path[count])
            goalError, y = PotentialFieldPlanner.fk.forward(np.reshape(goal, ((7,1))))
            positionError = np.zeros((1,7))

            #calculate the distance between the two errors to see if it's close 
            for i in range(7): 
                positionError = PotentialFieldPlanner.q_distance(checkError[i,:], goalError[i,:])
                errorValue = np.linalg.norm(positionError)

            e = 0.5
            jointPositions, jointPosAoe = PotentialFieldPlanner.fk.forward(q_path[count])
            count+=1
            match = False
            collide = False
            #joints 1 and 2 should be in  the same pos no mather what just different orinetations
            ok = [False, False, False, False, False]

            joint3Diff = PotentialFieldPlanner.q_distance(jointPositions[2,:], goalPosition[2,:])
            joint4Diff = PotentialFieldPlanner.q_distance(jointPositions[3,:], goalPosition[3,:])
            joint5Diff = PotentialFieldPlanner.q_distance(jointPositions[4,:], goalPosition[4,:])
            joint6Diff = PotentialFieldPlanner.q_distance(jointPositions[5,:], goalPosition[5,:])
            joint7Diff = PotentialFieldPlanner.q_distance(jointPositions[6,:], goalPosition[6,:])

            for i in ok: 
                if ok[i] <= e: 
                    ok[i] = True

            if PotentialFieldPlanner.q_distance(q,goal) < e: 
                match = True 

            # TODO: this is how to change your joint angles 
            
            # Termination Conditions
            if match: # TODO: check termination conditions
                break # exit the while loop if conditions are met!
            
            #check if they collied 
            if len(q_path) > 1: 
                newPosition, newPosAoe = PotentialFieldPlanner.fk.forward(q_path[count-1])

                collide = False 
                for i in range(len(map_struct.obstacles)): 
                    for j in range(7): 
                        collided = detectCollision(np.reshape(newPosition[j,:],(1,3)),np.reshape(newPosition[j+1,:],(1,3)),map_struct.obstacles[i])[0]
                        if collided: 
                            print("collided")
                            collide = True
            
            if collide: 
                break

            


            # YOU NEED TO CHECK FOR COLLISIONS WITH OBSTACLES
            # TODO: Figure out how to use the provided function 

            # YOU MAY NEED TO DEAL WITH LOCAL MINIMA HERE
            # TODO: when detect a local minima, implement a random walk
            
            ## END STUDENT CODE

        return q_path

################################
## Simple Testing Environment ##
################################

if __name__ == "__main__":

    np.set_printoptions(suppress=True,precision=5)

    planner = PotentialFieldPlanner()
    
    # inputs 
    map_struct = loadmap("../maps/map1.txt")
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    
    # potential field planning
    q_path = planner.plan(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    
    # show results
    for i in range(q_path.shape[0]):
        error = PotentialFieldPlanner.q_distance(q_path[i, :], goal)
        print('iteration:',i,' q =', q_path[i, :], ' error={error:3.4f}'.format(error=error))

    print("q path: ", q_path)
