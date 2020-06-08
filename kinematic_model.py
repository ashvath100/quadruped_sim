
import numpy as np
import time


import math



L = 80/2 #length of robot
W = 80/2 
L_COXA = 30 #coxa length
L_FEMUR = 60 #femur length
L_TIBIA = 140 #tibia length





class robotKinematics:
    def __init__(self):
        self.c = 0
        self.gait = [0,0,0,0]

    def gaitgen(self,speed,count,leg):
        print(count)
        tranTime = 98
        cycleTime = 0
        gaitLegNo = [0,2,6,4]
        pushSteps = 6
        stepsInCycle = 8
        if (speed[0] > 5 or speed[0] < -5) or (speed[1] > 5 or speed[1] < -5):
            if (count == gaitLegNo[leg]):
                self.gait[0] = 0
                self.gait[1] = 0
                self.gait[2] = -80
            # elif (((count == gaitLegNo[leg]+1) or (count == gaitLegNo[leg]-(stepsInCycle-1))) and (self.gait[2] < 0)):
            #     self.gait[0] = (speed[0]* 0.784 * pushSteps)/(2*stepsInCycle)
            #     self.gait[1] = (speed[1]* 0.784 * pushSteps)/(2*stepsInCycle)   
            #     self.gait[2] =  0
            # else:
            #     self.gait[0] = self.gait[0] - (speed[0]*cycleTime)/stepsInCycle
            #     self.gait[1] = self.gait[1] - (speed[1]*cycleTime)/stepsInCycle  
            #     self.gait[2] =  0
        else:
            self.gait[2]=0
            
        return self.gait            

    def bodyik(self, X , Y , Z, Xdist, Ydist,pos,rot):
        bodyRotX =  rot[0]  
        bodyRotY =  rot[1]        
        bodyRotZ =  rot[2]        
        bodyPosX =  pos[0]        
        bodyPosY =  pos[1] 
        bodyPosZ =  pos[2]             

        totaldist = np.array([X + Xdist + bodyPosX, Y + Ydist + bodyPosY])
        distBodyCenterFeet = np.sqrt(totaldist[0]**2 + totaldist[1]**2)
        AngleBodyCenter = np.arctan2(totaldist[1], totaldist[0])
        rolly = np.tan(bodyRotY * np.pi/180) * totaldist[0]
        pitchy = np.tan(bodyRotX * np.pi/180) * totaldist[1]

        ansx = np.cos(AngleBodyCenter + (bodyRotZ * np.pi/180)) * distBodyCenterFeet - totaldist[0] + bodyPosX
        ansy = np.sin(AngleBodyCenter + (bodyRotZ * np.pi/180)) * distBodyCenterFeet - totaldist[1] + bodyPosY
        ansz = rolly+pitchy + bodyPosZ
        ans = np.array([ ansx, ansy ,ansz])

        return ans

    def legik(self,X , Y , Z):
        coxa = math.atan2(X,Y) 
        trueX = np.sqrt(X**2+ Y**2 ) - L_COXA
        im = np.sqrt(trueX**2 + Z**2)

        q1 = -math.atan2(Z,trueX)
        d1 = L_FEMUR**2 - L_TIBIA**2 + im**2
        d2 = 2*L_FEMUR*im
        q2 = math.acos(d1/d2)
        femur = (q1+q2) 

        d1 = L_FEMUR**2 - im**2 + L_TIBIA**2
        d2 = 2*L_TIBIA*L_FEMUR
        tibia = (math.acos(d1/d2)-1.57) 
        ang = np.array([ coxa, -femur ,-tibia])
        return ang

    def doik(self,pos,rot,speed):

        endpoints1 = np.array([np.cos(45/180*np.pi)*(L_COXA + L_FEMUR), np.sin(45/180*np.pi)*(L_COXA + L_FEMUR),      L_TIBIA])
        endpoints2 = np.array([np.cos(45/180*np.pi)*(L_COXA + L_FEMUR), np.sin(-45/180*np.pi)*(L_COXA + L_FEMUR),      L_TIBIA])
        endpoints3 = np.array([-np.cos(45/180*np.pi)*(L_COXA + L_FEMUR), np.sin(-45/180*np.pi)*(L_COXA + L_FEMUR),      L_TIBIA])
        endpoints4 = np.array([-np.cos(45/180*np.pi)*(L_COXA + L_FEMUR), np.sin(45/180*np.pi)*(L_COXA + L_FEMUR),      L_TIBIA])
        
    
        gait = self.gaitgen(speed,self.c,0)    
        ans1 = self.bodyik(endpoints1[0] + gait[0], endpoints1[1] + gait[1], endpoints1[2] + gait[2], L, W,pos,rot)
        angles1 = self.legik(endpoints1[0]+ans1[0] + gait[0],endpoints1[1]+ans1[1] + gait[1], endpoints1[2]+ans1[2] + gait[2])

        gait = self.gaitgen(speed,self.c,1)
        ans2 = self.bodyik(endpoints2[0] + gait[0], endpoints2[1] + gait[1], endpoints2[2] + gait[2], L, -W,pos,rot)
        angles2 = self.legik(endpoints2[0]+ans2[0] + gait[0],endpoints2[1]+ans2[1] + gait[1], endpoints2[2]+ans2[2] + gait[2])

        gait = self.gaitgen(speed,self.c,0)
        ans3 = self.bodyik(endpoints3[0] + gait[0], endpoints3[1] + gait[1], endpoints3[2] + gait[2], -L, -W,pos,rot)
        angles3 = self.legik(endpoints3[0]+ans3[0] + gait[0],endpoints3[1]+ans3[1] + gait[1], endpoints3[2]+ans3[2] + gait[2])

        gait = self.gaitgen(speed,self.c,0)
        ans4 = self.bodyik(endpoints4[0] + gait[0], endpoints4[1] + gait[1], endpoints4[2] + gait[2], -L, W,pos,rot)
        angles4 = self.legik(endpoints4[0]+ans4[0] + gait[0],endpoints4[1]+ans4[1] + gait[1], endpoints4[2]+ans4[2] + gait[2])

        self.c = (self.c+1) % 8
        return(angles1 ,angles4,angles3,angles2)
