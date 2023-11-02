import numpy as np 
from lib.calcJacobian import calcJacobian

def FK_velocity(q_in, dq):
    """
    :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    :param dq: 1 x 7 vector corresponding to the joint velocities.
    :return:
    velocity - 6 x 1 vector corresponding to the end effector velocities.    
    """

    ## STUDENT CODE GOES HERE
    # v = J * dq
    # J is a 6x7 matrix and dq is a 1x7 matrix 
    # 6x7 * 7x1

    J = calcJacobian(q_in)
    q_trans = np.transpose(q_in)
    velocity = np.zeros((6, 1))
    velocity = J @ np.transpose(q_in)
    


    return velocity

    