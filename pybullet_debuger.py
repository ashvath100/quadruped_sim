import pybullet as p
import time
import numpy as np
import sys

class pybulletDebug:
    def __init__(self):
       
       
        time.sleep(0.5)
        
        self.xId = p.addUserDebugParameter("x" , -40 , 40 , 0.)
        self.yId = p.addUserDebugParameter("y" , -40 , 40 , 0.)
        self.zId = p.addUserDebugParameter("z" , -40 , 40 , 0.)
        self.rollId = p.addUserDebugParameter("roll" , -6.28319 , 6.28319 , 0.)
        self.pitchId = p.addUserDebugParameter("pitch" , -6.28319 , 6.28319 , 0.)
        self.yawId = p.addUserDebugParameter("yaw" , -6.28319 , 6.28319 , 0.)
    
    def cam_and_robotstates(self , boxId):
                ####orientacion de la camara
        cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
       
     
        #read position from debug
        pos = np.array([p.readUserDebugParameter(self.xId),p.readUserDebugParameter(self.yId), p.readUserDebugParameter(self.zId)])
        rot =  np.array([p.readUserDebugParameter(self.rollId),p.readUserDebugParameter(self.pitchId), p.readUserDebugParameter(self.yawId)])
        
        return pos , rot
