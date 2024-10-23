import sys
import os
sys.path.append(os.getcwd())

from coppeliasim_zmqremoteapi_client import *
from lib.MobileRobot import MobileRobot
import math
import time
import csv
import threading


client = RemoteAPIClient()
sim = client.require('sim')
sim.startSimulation()
robot = MobileRobot('HEXA4S')

gDistances = [0] * 12
print(gDistances)
room = 'room2'

def RotateAlignTheWall(side):
    while True:
        side_front_dist = 0
        side_back_dist = 0
        if side == 'right':
            side_front_dist = gDistances[2]
            side_back_dist = gDistances[4]
        elif side == 'left':
            side_front_dist = gDistances[10]
            side_back_dist = gDistances[8]
        error = side_front_dist - side_back_dist

        if error > 50:
            error = 50
        
        P = 3 * error
        robot.Move(P, -P)
        
        if abs(error) < 2:
            break     

        time.sleep(0.1)
    
    robot.Move(0, 0)

def ScanRoom3():
    pass

robot.start_sensing()

start_time = time.time()    
while time.time() - start_time < 10:
    gDistances = robot.ultrasonic_data
    print(gDistances[0])
    time.sleep(0.1)

print('Stop sensing')
robot.stop_sensing()
time.sleep(1)
sim.stopSimulation()