import numpy as np 
import math


L = 80 #length of robot
W = 80 
L_COXA = 30 #coxa length
L_FEMUR = 60 #femur length
L_TIBIA = 140 #tibia length

bodyRotX = 10
bodyRotY = 0             
bodyRotZ = 0           
bodyPosX = 10     
bodyPosY = 0            
bodyPosZ = 0  

endpoints1 = np.array([np.cos(45/180*np.pi)*(L_COXA + L_FEMUR), np.sin(45/180*np.pi)*(L_COXA + L_FEMUR),      L_TIBIA])
endpoints2 = np.array([np.cos(45/180*np.pi)*(L_COXA + L_FEMUR), np.sin(-45/180*np.pi)*(L_COXA + L_FEMUR),      L_TIBIA])
endpoints3 = np.array([-np.cos(45/180*np.pi)*(L_COXA + L_FEMUR), np.sin(-45/180*np.pi)*(L_COXA + L_FEMUR),      L_TIBIA])
endpoints4 = np.array([-np.cos(45/180*np.pi)*(L_COXA + L_FEMUR), np.sin(45/180*np.pi)*(L_COXA + L_FEMUR),      L_TIBIA])
# print(endpoints1)
# print(endpoints2)
# print(endpoints3)
# print(endpoints4)
                      

def bodyik(X , Y , Z,   Xdist, Ydist):
    totaldist = np.array([X + Xdist + bodyPosX, Y + Ydist + bodyPosY])
    distBodyCenterFeet = np.sqrt(totaldist[0]**2 + totaldist[1]**2)
    AngleBodyCenter = np.arctan2(totaldist[1], totaldist[0])
    rolly = np.tan(bodyRotY * np.pi/180) * totaldist[0]
    pitchy = np.tan(bodyRotX * np.pi/180) * totaldist[1]

    ansx = np.cos(AngleBodyCenter + (bodyRotZ * np.pi/180)) * distBodyCenterFeet - totaldist[0] + bodyPosX
    ansy = np.sin(AngleBodyCenter + (bodyRotZ * np.pi/180)) * distBodyCenterFeet - totaldist[1] + bodyPosY
    ansz = rolly+pitchy + bodyPosZ
    ans = np.array([ ansx, ansy ,ansz])

    #print(ans)
    return ans

def legik(X , Y , Z):
    #print(X,Y,Z)
    coxa = math.atan2(Y,X) * 180/np.pi
    trueX = np.sqrt(X**2+ Y**2 ) - L_COXA
    IKSW = np.sqrt(trueX**2 + Z**2)

    IKA1 = math.atan(trueX / Z)
    d1 = L_TIBIA**2 - L_FEMUR**2 - IKSW**2
    d2 = -2*L_FEMUR*IKSW
    IKA2 = math.acos(d1/d2)
    femur = 90-(IKA1+IKA2) * 180/np.pi

    d1 = IKSW**2 - L_TIBIA**2 - L_FEMUR**2
    d2 = -2*IKSW*L_FEMUR
    tibia = 90-(math.acos(d1/d2)) * 180/np.pi
    ang = np.array([ coxa, femur ,tibia])
    return ang

def doik():

    ans1 = bodyik(endpoints1[0], endpoints1[1], endpoints1[2], L/2, W/2)
    angles1 = legik(endpoints1[0]+ans1[0],endpoints1[1]+ans1[1], endpoints1[2]+ans1[2])
    print(angles1)

    ans2 = bodyik(endpoints2[0], endpoints2[1], endpoints2[2], L/2, -W/2)
    angles2 = legik(endpoints2[0]+ans2[0],endpoints2[1]+ans2[1], endpoints2[2]+ans2[2])
    print(angles2)

    ans3 = bodyik(endpoints3[0], endpoints3[1], endpoints3[2], -L/2, -W/2)
    angles3 = legik(endpoints3[0]+ans3[0],endpoints3[1]+ans3[1], endpoints3[2]+ans3[2])
    print(angles3)

    ans4 = bodyik(endpoints4[0], endpoints4[1], endpoints4[2], -L/2, W/2)
    angles4 = legik(endpoints4[0]+ans4[0],endpoints4[1]+ans4[1], endpoints4[2]+ans4[2])
    print(angles4)


doik()