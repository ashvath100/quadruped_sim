from kinematic_model import robotKinematics
import numpy as np

robotKinematics = robotKinematics()

POS = np.array([0 , 0 , 0])
ROT = np.array([0 , 0 , 0])

FR_angles = robotKinematics.solve(POS , ROT)
print(FR_angles)
