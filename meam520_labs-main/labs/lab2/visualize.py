import sys
from math import pi
import numpy as np

import rospy
import roslib
import tf
import geometry_msgs.msg


from core.interfaces import ArmController

from lib.calculateIK6 import IK

rospy.init_node("visualizer")

# Using your solution code
ik = IK()

#########################
##  RViz Communication ##
#########################

tf_broad  = tf.TransformBroadcaster()
point_pubs = [
    rospy.Publisher('/vis/joint'+str(i), geometry_msgs.msg.PointStamped, queue_size=10)
    for i in range(7)
]

# Publishes the position of a given joint on the corresponding topic
def show_joint_position(joints,i):
    msg = geometry_msgs.msg.PointStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'world'
    msg.point.x = joints[i,0]
    msg.point.y = joints[i,1]
    msg.point.z = joints[i,2]
    point_pubs[i].publish(msg)

# Broadcasts a T0e as the transform from given frame to world frame
def show_pose(T0e,frame):
    tf_broad.sendTransform(
        tf.transformations.translation_from_matrix(T0e),
        tf.transformations.quaternion_from_matrix(T0e),
        rospy.Time.now(),
        frame,
        "world"
    )

# Uses the above methods to visualize the full results of your FK
def show_all_FK(state):
    q = state['position']
    joints, T0e = fk.forward(q)
    show_pose(T0e,"endeffector")
    for i in range(7):
        show_joint_position(joints,i)

# visualize the chosen IK target
def show_target(target):
    T0_target = np.vstack((np.hstack((target['R'], np.array([target['t']]).T)), np.array([[0, 0, 0, 1]])))
    show_pose(T0_target,"target")

#################
##  IK Targets ##
#################

# TODO: Try testing your own targets!

targets = [
    {
        'R': np.array([[ 9.31021595e-33,  8.65956056e-17, -1.00000000e+00],
                       [ 7.07106781e-01, -7.07106781e-01, -6.12323400e-17],
                       [-7.07106781e-01, -7.07106781e-01, -6.12323400e-17]
                       ]),
        't': np.array([2.56500000e-01, -3.67087878e-17, 6.43500000e-01])

    },
    {
        'R': np.array([[ 8.65956056e-17, -9.81389106e-33, -1.00000000e+00],
                       [-7.07106781e-01, -7.07106781e-01, -6.12323400e-17],
                       [-7.07106781e-01,  7.07106781e-01, -6.12323400e-17],
                       ]),
        't': np.array([2.56500000e-01, -3.67087878e-17, 6.43500000e-01])

    },
]

####################
## Test Execution ##
####################

if __name__ == "__main__":

    if len(sys.argv) < 1:
        print("usage:\n\tpython visualize.py IK")
        exit()

    arm = ArmController(on_state_callback=show_all_FK)

    if sys.argv[1] == 'IK':

        # Iterates thrugh the given targets, using your IK solution
        # Try editing the targets list above to do more testing!
        for i, target in enumerate(targets):
            print("Moving to target " + str(i) + "...")
            show_target(target)
            solutions = ik.panda_ik(target)
            q = solutions[0,:] # choose the first of multiple solutions
            arm.safe_move_to_position(q)
            if i < len(targets) - 1:
                input("Press Enter to move to next target...")

    else:
        print("invalid option")
