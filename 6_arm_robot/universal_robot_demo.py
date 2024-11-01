import os
import sys
sys.path.append(os.getcwd())

from coppeliasim_zmqremoteapi_client import *
from lib.ArmRobot import UniversalRobot
import time

# 
client = RemoteAPIClient()
sim = client.require('sim')
sim.startSimulation()

# initialize robot instance
armRobot = UniversalRobot('UR3')

 # Get object position
initPos = armRobot.ReadPosition()
print(f"robot initial position: {initPos}")

# Get target position
targetPosition1 = armRobot.GetObjectPosition('target1')
print(f"position of target 1: {targetPosition1}")

targetPosition2 = armRobot.GetObjectPosition('target2')
print(f"position of target 2: {targetPosition2}")

targetPosition3 = armRobot.GetObjectPosition('target3')
print(f"position of target 3: {targetPosition3}")

for i in range(4):
    armRobot.MoveL(targetPosition1, 50)
    armRobot.MoveL(targetPosition3, 50)
    armRobot.MoveL(targetPosition2, 50)
    armRobot.MoveL(targetPosition3, 50)

print('Pose completed!')
time.sleep(3)
sim.stopSimulation()