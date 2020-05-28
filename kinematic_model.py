
import numpy as np
import math

class robotKinematics:
 

    def solve(self,POS ,ROT):
        BodySide = 80 #length of robot 
        CoxaLength = 30 #coxa length
        FemurLength = 60 #femur length
        TibiaLength = 140 #tibia length       
        pos = POS
        rot = ROT

        BodyCenterOffset = BodySide/2


        bodytoFR = np.array([ BodyCenterOffset, BodyCenterOffset ])
        bodytoFL = np.array([ -BodyCenterOffset, BodyCenterOffset ])
        bodytoBR = np.array([ BodyCenterOffset, -BodyCenterOffset ])
        bodytoBL = np.array([-BodyCenterOffset, -BodyCenterOffset ])


        initialpos = np.matrix([[np.cos(45/180*np.pi)*(CoxaLength + FemurLength), np.sin(45/180*np.pi)*(CoxaLength + FemurLength),      TibiaLength],
                                [np.cos(45/180*np.pi)*(CoxaLength + FemurLength), np.sin(-45/180*np.pi)*(CoxaLength + FemurLength),     TibiaLength],
                                [-np.cos(45/180*np.pi)*(CoxaLength + FemurLength), np.sin(-45/180*np.pi)*(CoxaLength + FemurLength),     TibiaLength],
                                [-np.cos(45/180*np.pi)*(CoxaLength + FemurLength), np.sin(45/180*np.pi)*(CoxaLength + FemurLength),     TibiaLength]]) 

        totaldist = np.matrix([[initialpos[0,0] + bodytoFR[0] + pos[0], initialpos[0,1] + bodytoFR[1] + pos[1]],
                            [initialpos[1,0] + bodytoBR[0] + pos[0], initialpos[1,1] + bodytoBR[1] + pos[1]],
                            [initialpos[2,0] + bodytoBL[0] + pos[0], initialpos[2,1] + bodytoBL[1] + pos[1]],
                            [initialpos[3,0] + bodytoFL[0] + pos[0], initialpos[3,1] + bodytoFL[1] + pos[1]]])

        #print(totaldist)

        distBodyCenterFeet = np.matrix([[np.sqrt(totaldist[0,0]**2 + totaldist[0,1]**2)],
                            [np.sqrt(totaldist[1,0]**2 + totaldist[1,1]**2)],
                            [np.sqrt(totaldist[2,0]**2 + totaldist[2,1]**2)],
                            [np.sqrt(totaldist[3,0]**2 + totaldist[3,1]**2)]])
                        
                    

        AngleBodyCenter = np.matrix([[np.arctan2(totaldist[0,1], totaldist[0,0])],
                            [np.arctan2(totaldist[1,1], totaldist[1,0])],
                            [np.arctan2(totaldist[2,1], totaldist[2,0])],
                            [np.arctan2(totaldist[3,1], totaldist[3,0])]])

        #print(AngleBodyCenter)

        rolly = np.array([[np.tan(rot[1] * np.pi/180) * totaldist[0,0]],
                            [np.tan(rot[1] * np.pi/180) * totaldist[1,0]],
                            [np.tan(rot[1] * np.pi/180) * totaldist[2,0]],
                            [np.tan(rot[1] * np.pi/180) * totaldist[3,0]]])
        #print(rolly)

        pitchy = np.array([[np.tan(rot[0] * np.pi/180) * totaldist[0,1]],
                            [np.tan(rot[0] * np.pi/180) * totaldist[1,1]],
                            [np.tan(rot[0] * np.pi/180) * totaldist[2,1]],
                            [np.tan(rot[0] * np.pi/180) * totaldist[3,1]]])
        #print(pitchy)


        bodyik = np.array([[np.cos(AngleBodyCenter[0,0] + (rot[2] * np.pi/180)) * distBodyCenterFeet[0] - totaldist[0,0],   np.sin(AngleBodyCenter[0,0] + (rot[2] * np.pi/180)) * distBodyCenterFeet[0] - totaldist[0,1],    rolly[0]+pitchy[0]],
                [np.cos(AngleBodyCenter[1,0] + (rot[2] * np.pi/180)) * distBodyCenterFeet[1] - totaldist[1,0],     np.sin(AngleBodyCenter[1,0] + (rot[2] * np.pi/180)) * distBodyCenterFeet[1] - totaldist[1,1],    rolly[1]+pitchy[1]],
                [np.cos(AngleBodyCenter[2,0] + (rot[2] * np.pi/180)) * distBodyCenterFeet[2] - totaldist[2,0],     np.sin(AngleBodyCenter[2,0] + (rot[2] * np.pi/180)) * distBodyCenterFeet[2] - totaldist[2,1],    rolly[2]+pitchy[2]],
                [np.cos(AngleBodyCenter[3,0] + (rot[2] * np.pi/180)) * distBodyCenterFeet[3] - totaldist[3,0],     np.sin(AngleBodyCenter[3,0] + (rot[2] * np.pi/180)) * distBodyCenterFeet[3] - totaldist[3,1],    rolly[3]+pitchy[3]]]) 

        #print(initialpos[0,0])

        newpos = np.array([[(pos[0] + initialpos[0,0] + bodyik[0][0]), (pos[1] + initialpos[0,1] + bodyik[0][1]), (pos[2] + initialpos[0,2] + bodyik[0][2])],
                            [(pos[0] + initialpos[1,0] + bodyik[1][0]), (pos[1] + initialpos[1,1] + bodyik[1][1]),  (pos[2] + initialpos[1,2] + bodyik[1][2])],
                            [(pos[0] + initialpos[2,0] + bodyik[2][0]), (pos[1] + initialpos[2,1] + bodyik[2][1]),  (pos[2] + initialpos[2,2] + bodyik[2][2])],
                            [(pos[0] + initialpos[3,0] + bodyik[3][0]), (pos[1] + initialpos[3,1] + bodyik[3][1]),  (pos[2] + initialpos[3,2] + bodyik[3][2])]]) 

        #print(newpos)


        coxaFeetDist = np.matrix([[math.sqrt((newpos[0][0]**2) + (newpos[0][1]**2))],
                            [math.sqrt((newpos[1][0]**2) + (newpos[1][1]**2))],
                            [math.sqrt((newpos[2][0]**2) + (newpos[2][1]**2))],
                            [math.sqrt((newpos[3][0]**2) + (newpos[3][1]**2))]])

        IKSW = np.matrix([[math.sqrt(((coxaFeetDist[0] - CoxaLength)**2) + (newpos[0][2]**2))],
                            [math.sqrt(((coxaFeetDist[1] - CoxaLength)**2) + (newpos[1][2]**2))],
                            [math.sqrt(((coxaFeetDist[2] - CoxaLength)**2) + (newpos[2][2]**2))],
                            [math.sqrt(((coxaFeetDist[3] - CoxaLength)**2) + (newpos[3][2]**2))]])

        IKA1 = np.matrix([[math.atan((coxaFeetDist[0] - CoxaLength) / (newpos[0][2]))],
                            [math.atan((coxaFeetDist[1] - CoxaLength) / (newpos[1][2]))],
                            [math.atan((coxaFeetDist[2] - CoxaLength) / (newpos[2][2]))],
                            [math.atan((coxaFeetDist[3] - CoxaLength) / (newpos[3][2]))]])

            
        IKA2 = np.array([[math.acos(((TibiaLength**2) - (FemurLength**2) - (IKSW[0]**2)) / (-2 * IKSW[0] * FemurLength))],
                            [math.acos(((TibiaLength**2) - (FemurLength**2) - (IKSW[1]**2)) / (-2 * IKSW[1] * FemurLength))],
                            [math.acos(((TibiaLength**2) - (FemurLength**2) - (IKSW[2]**2)) / (-2 * IKSW[2] * FemurLength))],
                            [math.acos(((TibiaLength**2) - (FemurLength**2) - (IKSW[3]**2)) / (-2 * IKSW[3] * FemurLength))]])

        T_angle = np.array([[math.acos(((IKSW[0]**2) - (TibiaLength**2) - (FemurLength**2)) / (-2 * IKSW[0] * FemurLength))],
                            [math.acos(((IKSW[1]**2) - (TibiaLength**2) - (FemurLength**2)) / (-2 * IKSW[1] * FemurLength))],
                            [math.acos(((IKSW[2]**2) - (TibiaLength**2) - (FemurLength**2)) / (-2 * IKSW[2] * FemurLength))],
                            [math.acos(((IKSW[3]**2) - (TibiaLength**2) - (FemurLength**2)) / (-2 * IKSW[3] * FemurLength))]])

        iktibia = np.array([[90-(T_angle[0])* 180/np.pi],
                            [90-(T_angle[1])* 180/np.pi],
                            [90-(T_angle[2])* 180/np.pi],
                            [90-(T_angle[3])* 180/np.pi]])

        ikfemur = np.array([[90-(IKA1[0] + IKA2[0])* 180/np.pi],
                            [90-(IKA1[1] + IKA2[1])* 180/np.pi],
                            [90-(IKA1[2] + IKA2[2])* 180/np.pi],
                            [90-(IKA1[3] + IKA2[3])* 180/np.pi]])

        ikcoxa = np.array([[(math.atan2(newpos[0][1],newpos[0][0]) * 180/np.pi)],
                                    [math.atan2(newpos[1][1],newpos[1][0]) * 180/np.pi],
                                    [math.atan2(newpos[2][1],newpos[2][0]) * 180/np.pi],
                                    [math.atan2(newpos[3][1],newpos[3][0]) * 180/np.pi]]) 
        # print(iktibia)
        # print(ikfemur)
        # print(ikcoxa)
        
        FR_angles = np.array([(ikcoxa[0]-45)*(np.pi/180),ikfemur[0]*(np.pi/180),iktibia[0]*(np.pi/180)])

        BL_angles = np.array([(ikcoxa[1]+45)*(np.pi/180),ikfemur[1]*(np.pi/180),iktibia[1]*(np.pi/180)])
        BR_angles = np.array([(ikcoxa[2]+135)*(np.pi/180),ikfemur[2]*(np.pi/180),iktibia[2]*(np.pi/180)])
        FL_angles = np.array([(ikcoxa[3]-135)*(np.pi/180),ikfemur[3]*(np.pi/180),iktibia[3]*(np.pi/180)])
        return FR_angles ,  BL_angles, BR_angles, FL_angles
