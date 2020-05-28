

import pybullet as p
import numpy as np
import time
import pybullet_data
from pybullet_debuger import pybulletDebug  
from kinematic_model import robotKinematics

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)


cubeStartPos = [0,0,2]
FixedBase = False #if fixed no plane is imported
if (FixedBase == False):
    p.loadURDF("plane.urdf")
boxId = p.loadURDF("quad.urdf",cubeStartPos, useFixedBase=FixedBase)

jointIds = []
paramIds = [] 
time.sleep(0.5)
for j in range(p.getNumJoints(boxId)):
#    p.changeDynamics(boxId, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(boxId, j)
    print(info)
    jointName = info[1]
    jointType = info[2]
    jointIds.append(j)
    
footFR_index = 3
footFL_index = 7
footBR_index = 11
footBL_index = 15   
pybulletDebug = pybulletDebug()
robotKinematics = robotKinematics()




p.setRealTimeSimulation(1)
    
while(True):
    lastTime = time.time()
    POS , ROT = pybulletDebug.cam_and_robotstates(boxId)  
    FR_angles ,  BL_angles, BR_angles, FL_angles = robotKinematics.solve(POS , ROT)
        

    for i in range(0, footFR_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, FR_angles[i - footFR_index])
    for i in range(footFR_index + 1, footFL_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, FL_angles[i - footFL_index])
    for i in range(footFL_index + 1, footBR_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, BR_angles[i - footBR_index])
    for i in range(footBR_index + 1, footBL_index):
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, BL_angles[i - footBL_index])
    
    
#    print(time.time() - lastTime)
p.disconnect()