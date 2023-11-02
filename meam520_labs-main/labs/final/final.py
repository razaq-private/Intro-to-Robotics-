#Sharanya latest dynamic code
#Still doesn't pick up the block though
#hopefully no more collisions
import sys
import numpy as np
from copy import deepcopy
from math import pi

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds
from lib.calculateFK import FK
from lib.calculateIK6 import IK


if __name__ == "__main__":
    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    arm = ArmController()
    detector = ObjectDetector()

    start_position = np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/8, 0.75344866])
    arm.safe_move_to_position(start_position) # on your mark!

    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
        yplat = 0.731-0.9
        init1 = 0.35
        blockplat = 1.069-0.9
    else:
        yplat = 0.9-0.731
        init1 = -0.35
        blockplat = 0.9-1.069
        print("**  RED TEAM  **")
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!

    # STUDENT CODE HERE

    # get the transform from camera to panda_end_effector
        # get the transform from camera to panda_end_effector
    
    # FK = FK()
    # IK = IK()
    # current_q = []
    # initial_poses = []
    # #current_q.append(np.array([init1, -0.56012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/6, 0.75344866]))
    # #initial_poses.append(-2.12532082e-01 -2.82066550e-01  4.29352544e-05 -2.33490113e+00, 0.00000000e+00  2.35281180e+00  0.78539816339)
    # blockplatframe = np.array([[1, 0, 0, 0.562],[0, -1, 0, blockplat],[0, 0, -1, 0.6],[0, 0, 0, 1]])
    # target = {'R': blockplatframe[0:3, 0:3], 't': blockplatframe[0:3, 3]}
    # invoutput = IK.panda_ik(target)
    # current_q.append(invoutput[0])
    # arm.safe_move_to_position(current_q[-1])
    # H_ee_camera = detector.get_H_ee_camera()

    # platformframe = np.array([[1, 0, 0, 0.562],[0, -1, 0, yplat],[0, 0, -1, 0.2],[0, 0, 0, 1]])
    # # Detect some blocks...
    # xrotmatrixpos90 = [[1, 0, 0, 0],[0,np.cos(pi/2),-np.sin(pi/2),0],[0, np.sin(pi/2), np.cos(pi/2),0],[0,0,0,1]]
    # xrotmatrixneg90 = [[1, 0, 0, 0],[0,np.cos(-pi/2),-np.sin(-pi/2),0],[0, np.sin(-pi/2), np.cos(-pi/2),0],[0,0,0,1]]
    # yrotmatrixpos90 = [[np.cos(pi/2), 0, np.sin(pi/2),0],[0,1,0,0],[-np.sin(pi/2), 0, np.cos(pi/2),0],[0,0,0,1]]
    # yrotmatrixneg90 = [[np.cos(-pi/2), 0, np.sin(-pi/2),0],[0,1,0,0],[-np.sin(-pi/2), 0, np.cos(-pi/2),0],[0,0,0,1]]

    # yrotmatrix180 = [[np.cos(pi), 0, np.sin(pi),0],[0,1,0,0],[-np.sin(pi), 0, np.cos(pi),0],[0,0,0,1]]
    # zrotmatrix = [[np.cos(pi/2),-np.sin(pi/2),0, 0],[np.sin(pi/2), np.cos(pi/2), 0, 0],[0,0,1,0],[0,0,0,1]]
    # i = 0
    # for (name, pose) in detector.get_detections():
    #      print(name,'\n',pose)
    #      joint_pos, T0e = FK.forward(current_q[-1])
    #      cameratoworld = np.matmul(T0e,H_ee_camera)
    #      invinput = np.matmul(cameratoworld,pose)
    #      newinvinput = np.matmul(cameratoworld,pose)
    #      add = newinvinput[2,3] + 0.2
    #      print('add',add)
    #      newinvinput[2,3] = add
    #      print('input',invinput)
    #      print('new input', newinvinput)
    #      if invinput[2,2] < -1.05 or invinput[2,2] > -0.95:
    #          if invinput[2,0] > -1.05 and invinput[2,0] < -0.95:
    #              invinput = np.matmul(invinput,yrotmatrixpos90)
    #              print('new matrix',invinput)
    #          if invinput[2,1] > -1.05 and invinput[2,1] < -0.95:
    #              invinput = np.matmul(invinput,xrotmatrixneg90)
    #              print('new matrix',invinput)
    #          if invinput[2,2] < 1.05 and invinput[2,2] > 0.95:
    #              invinput = np.matmul(invinput,yrotmatrix180)
    #              print('new matrix',invinput)
    #          if invinput[2,1] < 1.05 and invinput[2,1] > 0.95:
    #              invinput = np.matmul(invinput,xrotmatrixpos90)
    #              print('new matrix',invinput)
    #          if invinput[2,0] < 1.05 and invinput[2,0] > 0.95:
    #              invinput = np.matmul(invinput,yrotmatrixneg90)
    #              print('new matrix',invinput)
    #      target = {'R': invinput[0:3, 0:3], 't': invinput[0:3, 3]}
    #      invoutput = IK.panda_ik(target)
    #      print('output',invoutput)
    #      if len(invoutput) == 0:
    #          invinput = np.matmul(invinput,zrotmatrix)
    #          print('new matrix',invinput)
    #          target = {'R': invinput[0:3, 0:3], 't': invinput[0:3, 3]}
    #          invoutput = IK.panda_ik(target)
    #          print('output',invoutput)
    #      if newinvinput[2,2] < -1.05 or newinvinput[2,2] > -0.95:
    #          if newinvinput[2,0] > -1.05 and newinvinput[2,0] < -0.95:
    #              newinvinput = np.matmul(newinvinput,yrotmatrixpos90)
    #              print('new matrix',newinvinput)
    #          if newinvinput[2,1] > -1.05 and newinvinput[2,1] < -0.95:
    #              newinvinput = np.matmul(newinvinput,xrotmatrixneg90)
    #              print('new matrix',newinvinput)
    #          if newinvinput[2,2] < 1.05 and newinvinput[2,2] > 0.95:
    #              newinvinput = np.matmul(newinvinput,yrotmatrix180)
    #              print('new matrix',newinvinput)
    #          if newinvinput[2,1] < 1.05 and newinvinput[2,1] > 0.95:
    #              newinvinput = np.matmul(newinvinput,xrotmatrixpos90)
    #              print('new matrix',newinvinput)
    #          if newinvinput[2,0] < 1.05 and newinvinput[2,0] > 0.95:
    #              newinvinput = np.matmul(newinvinput,yrotmatrixneg90)
    #              print('new matrix',newinvinput)
    #      target = {'R': newinvinput[0:3, 0:3], 't': newinvinput[0:3, 3]}
    #      newinvoutput = IK.panda_ik(target)
    #      print('output',newinvoutput)
    #      if len(newinvoutput) == 0:
    #          newinvinput = np.matmul(newinvinput,zrotmatrix)
    #          print('new matrix',newinvinput)
    #          target = {'R': newinvinput[0:3, 0:3], 't': newinvinput[0:3, 3]}
    #          newinvoutput = IK.panda_ik(target)
    #          print('output',newinvoutput)
    #      joint7opts = np.zeros((4,7))
    #      newq = invoutput[0]
    #      for i in range(4):
    #          joint7 = newq[6]
    #          if joint7 < 0:
    #              newq[6] = joint7 + (pi/2 * i)
    #              joint7opts[i,:] = newq
    #          if joint7 > 0:
    #              newq[6] = joint7 - (pi/2 * i)
    #              joint7opts[i,:] = newq
    #      print('opts:', joint7opts)
    #      norms = np.zeros((4,1))
    #      for i in range(4):
    #         print('norms',np.linalg.norm(current_q[-1]-joint7opts[i]))
    #         norms[i] = np.linalg.norm(current_q[-1]-joint7opts[i])
    #      invoutput = joint7opts[np.where(norms ==min(norms))[0][0]]
    #      print(invoutput)

    #      joint7opts = np.zeros((4,7))
    #      newq = newinvoutput[0]
    #      print(newq)
    #      for i in range(4):
    #          joint7 = newq[6]
    #          if joint7 < 0:
    #              newq[6] = joint7 + (pi/2 * i)
    #              joint7opts[i,:] = newq
    #          if joint7 > 0:
    #              newq[6] = joint7 - (pi/2 * i)
    #              joint7opts[i,:] = newq
    #      print('opts:', joint7opts)
    #      norms = np.zeros((4,1))
    #      for i in range(4):
    #         print('norms',np.linalg.norm(current_q[-1]-joint7opts[i]))
    #         norms[i] = np.linalg.norm(current_q[-1]-joint7opts[i])
    #      newinvoutput = joint7opts[np.where(norms ==min(norms))[0][0]]
    #      print(newinvoutput)

    #      if len(invoutput) >0:
    #          print(current_q[-1])
    #          arm.exec_gripper_cmd(0.09, 0)
    #          print('opened')
    #          current_q.append(newinvoutput)
    #          arm.safe_move_to_position(current_q[-1])
    #          print('moved towards')
    #          current_q.append(invoutput)
    #          arm.safe_move_to_position(current_q[-1])
    #          print('moved')
    #          arm.exec_gripper_cmd(0.045, 2)
    #          print('gripped')
    #          movearmup = current_q[-1]
    #          movearmup[1] = movearmup[1]-0.3
    #          movearmup[6] = 1.07750781
    #          current_q.append(movearmup)
    #          print('next q to move up',current_q[-1])
    #          arm.safe_move_to_position(current_q[-1])
    #          print('moved')
    #          platformframe[2,3] = 0.25 + (0.06*i)
    #          target_podium = {'R': platformframe[0:3, 0:3], 't': platformframe[0:3, 3]}
    #          target_podium_config = IK.panda_ik(target_podium)
    #          current_q.append(target_podium_config[0])
    #          print('config',target_podium_config[0])
    #          arm.safe_move_to_position(current_q[-1])
    #          print('moved')
    #          arm.exec_gripper_cmd(0.09, 0)
    #          print('let go')
    #          movearmup = current_q[-1]
    #          movearmup[1] = movearmup[1]-0.3
    #          current_q.append(movearmup)
    #          print('next q to move up', current_q[-1])
    #          arm.safe_move_to_position(current_q[-1])
    #          movearmformore = current_q[0]
    #          #movearmformore[0] = movearmformore[0]-0.642
    #          current_q.append(movearmformore)
    #          print('next q to move back', current_q[-1])
    #          arm.safe_move_to_position(current_q[-1])
    #          i = i+1


    FK = FK()
    IK = IK()
    current_q = []
    current_q.append(np.array([1.5, -0.96012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/6, 0.75344866]))     #Can see all blocks in this position
    H_ee_camera = detector.get_H_ee_camera()
    arm.safe_move_to_position(current_q[-1])
    platformframe = np.array([[1, 0, 0, 0.562],[0, -1, 0, yplat],[0, 0, -1, 0.2],[0, 0, 0, 1]])

    if detector.get_detections():
        print('I see cubes!!')
        for (name, pose) in detector.get_detections():
            print(name,'\n',pose)


    # Detect some blocks...
    xrotmatrixpos90 = [[1, 0, 0, 0],[0,np.cos(pi/2),-np.sin(pi/2),0],[0, np.sin(pi/2), np.cos(pi/2),0],[0,0,0,1]]
    xrotmatrixneg90 = [[1, 0, 0, 0],[0,np.cos(-pi/2),-np.sin(-pi/2),0],[0, np.sin(-pi/2), np.cos(-pi/2),0],[0,0,0,1]]
    yrotmatrixpos90 = [[np.cos(pi/2), 0, np.sin(pi/2),0],[0,1,0,0],[-np.sin(pi/2), 0, np.cos(pi/2),0],[0,0,0,1]]
    yrotmatrixneg90 = [[np.cos(-pi/2), 0, np.sin(-pi/2),0],[0,1,0,0],[-np.sin(-pi/2), 0, np.cos(-pi/2),0],[0,0,0,1]]

    yrotmatrix180 = [[np.cos(pi), 0, np.sin(pi),0],[0,1,0,0],[-np.sin(pi), 0, np.cos(pi),0],[0,0,0,1]]
    zrotmatrix = [[np.cos(pi/2),-np.sin(pi/2),0, 0],[np.sin(pi/2), np.cos(pi/2), 0, 0],[0,0,1,0],[0,0,0,1]]
    # i = 0
    # for (name, pose) in detector.get_detections():
    #      print(name,'\n',pose)
    #      joint_pos, T0e = FK.forward(current_q[-1])
    #      cameratoworld = np.matmul(T0e,H_ee_camera)
    #      invinput = np.matmul(cameratoworld,pose)
    #      print('input',invinput)
    #      if invinput[2,2] < -1.05 or invinput[2,2] > -0.95:
    #          if invinput[2,0] > -1.05 and invinput[2,0] < -0.95:
    #              invinput = np.matmul(invinput,yrotmatrixpos90)
    #              print('new matrix',invinput)
    #          if invinput[2,1] > -1.05 and invinput[2,1] < -0.95:
    #              invinput = np.matmul(invinput,xrotmatrixneg90)
    #              print('new matrix',invinput)
    #          if invinput[2,2] < 1.05 and invinput[2,2] > 0.95:
    #              invinput = np.matmul(invinput,yrotmatrix180)
    #              print('new matrix',invinput)
    #          if invinput[2,1] < 1.05 and invinput[2,1] > 0.95:
    #              invinput = np.matmul(invinput,xrotmatrixpos90)
    #              print('new matrix',invinput)
    #          if invinput[2,0] < 1.05 and invinput[2,0] > 0.95:
    #              invinput = np.matmul(invinput,yrotmatrixneg90)
    #              print('new matrix',invinput)    

                 
    #      target = {'R': invinput[0:3, 0:3], 't': invinput[0:3, 3]}

    #      print('target dict', target)
        #  invoutput = IK.panda_ik(target)
        #  print('output',invoutput)
        #  joint7opts = np.zeros((4,7))
        #  newq = invoutput[0]
        #  for i in range(4):
        #      joint7 = newq[6]
        #      newq[6] = joint7 + (pi/2 * i)
        #      joint7opts[i,:] = newq
        #  print('opts:', joint7opts)
        #  norms = np.zeros((4,1))
        #  for i in range(4):
        #     print('joint7opts[i]', joint7opts[i])
        #     print('current_q[-1]',current_q[-1])
        #     print('ith norm', np.linalg.norm(current_q[-1]-joint7opts[i]))
        #     norms[i] = np.linalg.norm(current_q[-1]-joint7opts[i])
        #     #if joint7opts[i][6] >=-pi/4 and joint7opts[i][6] <= 3*pi/4:
        #         #norms = joint7opts[i]
        #  print('norms',norms)
        #  print(min(norms))   
        #  #print(np.where(norms == )[0][0])
        #  ikinput = joint7opts[np.where(norms ==min(norms))[0][0]]
        #  #ikinput = norms 
        #  print(ikinput)
        #  if len(invoutput) >0:
        #      arm.exec_gripper_cmd(0.09, 0)
        #      current_q.append(ikinput)
        #      arm.safe_move_to_position(current_q[-1])
        #      print('moved')
        #      arm.exec_gripper_cmd(0.045, 2)
        #      print('gripped')
        #      movearmup = current_q[-1]
        #      movearmup[1] = movearmup[1]-0.3
        #      movearmup[6] = 1.07750781
        #      current_q.append(movearmup)
        #      print('next q to move up',current_q[-1])
        #      arm.safe_move_to_position(current_q[-1])
        #      print('moved')
        #      platformframe[2,3] = 0.25 + (0.06*i)
        #      target_podium = {'R': platformframe[0:3, 0:3], 't': platformframe[0:3, 3]}
        #      target_podium_config = IK.panda_ik(target_podium)
        #      current_q.append(target_podium_config[0])
        #      print('config',target_podium_config[0])
        #      arm.safe_move_to_position(current_q[-1])
        #      print('moved')
        #      arm.exec_gripper_cmd(0.08, 0)
        #      print('let go')
        #      movearmup = current_q[-1]
        #      movearmup[1] = movearmup[1]-0.3
        #      current_q.append(movearmup)
        #      print('next q to move up', current_q[-1])
        #      arm.safe_move_to_position(current_q[-1])
        #      movearmformore = current_q[0]
        #      #movearmformore[0] = movearmformore[0]-0.642
        #      current_q.append(movearmformore)
        #      print('next q to move back', current_q[-1])
        #      arm.safe_move_to_position(current_q[-1])
        #      i = i+1

    
    arm.exec_gripper_cmd(0.09, 0)
    # t = np.array([0, 0.685, 0.2025])
    R = np.array([[-1,0,0], [0,1,0], [0,0,-1]])  
    # R = np.array([[1,0,0], [0,-1,0], [0,0,-1]])    #detects leaving block
    # R = np.array([[0,1,0], [1,0,0], [0,0,-1]])   #Francesca
    t = np.array([0, 0.70, 0.3025])    #Just to estimate angular velocity
    target = {'R' : R, 't' : t}
    print(target)
    current_q = IK.panda_ik(target)
    print(current_q)
    arm.safe_move_to_position(current_q[-1])

    # H_ee_camera = detector.get_H_ee_camera()
    print('H_ee_camera matrix', H_ee_camera)
    for (name, pose) in detector.get_detections():
         print(name,'\n',pose)
        #  joint_pos, T0e = FK.forward(current_q[-1])
    
    # arm.exec_gripper_cmd(0.045, 2)
    # print('gripped')
    # movearmup = current_q[-1]
    # movearmup[1] = movearmup[1]-0.3
    # movearmup[6] = 1.07750781
    # current_q.append(movearmup)
    # print('next q to move up',current_q[-1])
    # arm.safe_move_to_position(current_q[-1])
    # arm.exec_gripper_cmd(0.08, 0)

    while(1):
        if detector.get_detections():
            joint_pos, T0e = FK.forward(current_q[-1])
            cameratoworld = np.matmul(T0e,H_ee_camera)
            invinput = np.matmul(cameratoworld,pose)
            print('input',invinput)
            if invinput[2,2] < -1.05 or invinput[2,2] > -0.95:
                if invinput[2,0] > -1.05 and invinput[2,0] < -0.95:
                    invinput = np.matmul(invinput,yrotmatrixpos90)
                    print('new matrix',invinput)
                if invinput[2,1] > -1.05 and invinput[2,1] < -0.95:
                    invinput = np.matmul(invinput,xrotmatrixneg90)
                    print('new matrix',invinput)
                if invinput[2,2] < 1.05 and invinput[2,2] > 0.95:
                    invinput = np.matmul(invinput,yrotmatrix180)
                    print('new matrix',invinput)
                if invinput[2,1] < 1.05 and invinput[2,1] > 0.95:
                    invinput = np.matmul(invinput,xrotmatrixpos90)
                    print('new matrix',invinput)
                if invinput[2,0] < 1.05 and invinput[2,0] > 0.95:
                    invinput = np.matmul(invinput,yrotmatrixneg90)
                    print('new matrix',invinput)    

                    
            target = {'R': invinput[0:3, 0:3], 't': invinput[0:3, 3]}

            print('target dict', target)
            invoutput = IK.panda_ik(target)
            print('output',invoutput)
            joint7opts = np.zeros((4,7))
            newq = invoutput[0]
            for i in range(4):
                joint7 = newq[6]
                newq[6] = joint7 + (pi/2 * i)
                joint7opts[i,:] = newq
            print('opts:', joint7opts)
            norms = np.zeros((4,1))
            for i in range(4):
                print('joint7opts[i]', joint7opts[i])
                print('current_q[-1]',current_q[-1])
                print('ith norm', np.linalg.norm(current_q[-1]-joint7opts[i]))
                norms[i] = np.linalg.norm(current_q[-1]-joint7opts[i])
                #if joint7opts[i][6] >=-pi/4 and joint7opts[i][6] <= 3*pi/4:
                    #norms = joint7opts[i]
            print('norms',norms)
            print(min(norms))   
            #print(np.where(norms == )[0][0])
            ikinput = joint7opts[np.where(norms ==min(norms))[0][0]]
            #ikinput = norms 
            print(ikinput)
            if len(invoutput) >0:
                arm.exec_gripper_cmd(0.09, 0)
                current_q = ikinput
                #  current_q.append(ikinput)           #Francesca's original
                arm.safe_move_to_position(current_q[-1])
                print('moved')
                arm.exec_gripper_cmd(0.045, 2)
                print('gripped')
                movearmup = current_q[-1]
                movearmup[1] = movearmup[1]-0.3
                movearmup[6] = 1.07750781
                current_q = movearmup
                #  current_q.append(movearmup)
                print('next q to move up',current_q[-1])
                arm.safe_move_to_position(current_q[-1])
                print('moved')
                platformframe[2,3] = 0.25 + (0.06*i)
                target_podium = {'R': platformframe[0:3, 0:3], 't': platformframe[0:3, 3]}
                target_podium_config = IK.panda_ik(target_podium)
                #  current_q.append(target_podium_config[0])
                current_q = target_podium_config[0]
                print('config',target_podium_config[0])
                arm.safe_move_to_position(current_q[-1])
                print('moved')
                arm.exec_gripper_cmd(0.08, 0)
                print('let go')
                movearmup = current_q[-1]
                movearmup[1] = movearmup[1]-0.3
                #  current_q.append(movearmup)
                current_q = movearmup
                print('next q to move up', current_q[-1])
                arm.safe_move_to_position(current_q[-1])
                movearmformore = current_q[0]
                #movearmformore[0] = movearmformore[0]-0.642
                current_q = movearmformore
                #  current_q.append(movearmformore)
                print('next q to move back', current_q[-1])
                arm.safe_move_to_position(current_q[-1])
                i = i+1


    # while(1):
    #     if detector.get_detections():
    #         print('here', detector.get_detections())
    #         print('type', type(detector.get_detections()))
    #         tracker_name = detector.get_detections()[0]
    #         print('I see cube ', tracker_name )
    #         start_time = time_in_seconds()
    #         break

    # while detector.get_detections:   #wait until you can't see anything
    #             continue

    # while(1):
    #     if detector.get_detections():
    #         name = detector.get_detections()[0]
            
    #         if name == tracker_name:
    #             print('I see the cube again! ', name)
    #             end_time = time_in_seconds()
    #             break

    # print('start time ', start_time, 'end time', end_time, 'duration', end_time - start_time)
        # if name == tracker