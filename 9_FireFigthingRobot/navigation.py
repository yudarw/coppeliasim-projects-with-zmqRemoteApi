import sys
import os
sys.path.append(os.getcwd())
from coppeliasim_zmqremoteapi_client import *
from lib.MobileRobot import Hexa4R
from lib.sick_tim310 import SickTIM310
import time
import numpy as np
import math

# Global Variable:
# parameters for pid rotation
Kp_rot = 3
Ki_rot = 0.001   
Kd_rot = 0.5
error_rot = 0
last_error_rot = 0
pid_rot = 0
sum_error_rot = 0

# parameters for pid translation
Kp_trans = 1
Ki_tans = 0
Kd_tans = 0
error_trans = 0
last_error_trans = 0
pid_trans = 0

wall_distance = 10      # 5 cm from the wall
Ts = 0.05               # Time sampling 
Sp = 80                # Initial speed

robot = None
lidar_sensor = None

# Implement the wall-following alrgorithm
def scan(side, dist):
    global sum_error_rot
    # Read distance
    #dist = robot.ReadUltrasonicSensors()
    side_front_dist = 0
    side_back_dist = 0
    side_dist = 0
    if side == 'right':
        side_front_dist = dist[2]
        side_back_dist = dist[4]
        side_dist = dist[3]
        
    elif side == 'left':
        side_front_dist = dist[10]
        side_back_dist = dist[8]
        side_dist = dist[9]
    
    # ----------------  First PID: control the orientation ---------------- #
    error_rot = side_front_dist - side_back_dist        
    
    if error_rot > 40:
        error_rot = 40
    elif error_rot < -40:
        error_rot = -40
        
    last_error_rot = error_rot
    sum_error_rot = sum_error_rot + error_rot 
    pid_rot = Kp_rot * error_rot + Ki_rot * sum_error_rot * Ts + Kd_rot * (error_rot - last_error_rot) / Ts 

    # ----------------  Second PID: control the distance from the wall ---------------- #
    error_trans = side_dist - wall_distance
    last_error_trans = error_trans
    pid_trans = Kp_trans * error_trans + Kd_tans * (error_trans - last_error_trans) / Ts

    #print(f'error_rot: {error_rot}, error_trans: {error_trans}')
    #print(f'dist from wall: {side_dist} cm')    

    robot.Move(Sp + pid_rot + pid_trans, Sp - pid_rot - pid_trans)

    # Check the front sensor, if the front sensor is too close rotate robot to the left
    if dist[0] < 13:
        robot.Rotate(-90, 90)
        WallAlign(side=side)
    # Check the front side sensor to detect an intersection
    # It will make the robot smoothly turn to the right.
    if dist[2] > 80:
        robot.Move(80, 20)
        sim.wait(5)

    #time.sleep(Ts)

# Align the body of the robot with the wall
def WallAlign(side):
    while True:
        dist = robot.ReadUltrasonicSensors()
        side_front_dist = 0
        side_back_dist = 0
        if side == 'right':
            side_front_dist = dist[2]
            side_back_dist = dist[4]
        elif side == 'left':
            side_front_dist = dist[10]
            side_back_dist = dist[8]
        
        error = side_front_dist - side_back_dist
        
        if error > 50:
            error = 50
        
        P = 2 * error
        robot.Move(P, -P)
        
        if abs(error) < 2:
            break
        
        time.sleep(0.05)
    robot.Move(0, 0)


def exit_room():

    while True:
        pass


def find_door():
    WallAlign('right')

    while True:
        distance = robot.ReadUltrasonicSensors()
        scan('right', distance)

        lidar = lidar_sensor.getAllDistances()
        lidar_front = np.mean(lidar[130:140])
        lidar_diagonal = np.mean(lidar[175:185])
        lidar_left = np.mean(lidar[220:230])
        print(f'Detect corner: L={lidar_left}, F={lidar_front}, D={lidar_diagonal}')
        
        if lidar_front > 120:
            #t = math.sqrt(lidar_diagonal * lidar_diagonal - lidar_left * lidar_left)
            #room_size = (lidar_left * t) / 2
            room_size = lidar_diagonal * lidar_left
            print(f's={lidar_diagonal}, a={lidar_left}, room_size={room_size}')
            
            
            if room_size > 6000 and room_size < 9000:
                print('Facing the door, and detect room 3')
            elif room_size < 2000:
                print('Detect ROOM2, facing door')
            else:
                print('Detect ROOM1, facing door')
            break

        # Room 4
        elif lidar_front > 80 and lidar_left < 70:
            #t = math.sqrt(lidar_diagonal * lidar_diagonal - lidar_left * lidar_left)
            #room_size = (lidar_left * t) / 2
            room_size = lidar_diagonal * lidar_left
            print(f's={lidar_diagonal}, a={lidar_left}, room_size={room_size}')

            if room_size < 2500:
                print('Facing the door, and detect room 4')
            else:
                print('Detect ROOM2, not facing DOOR')
            
            break

    print('Robot is facing to the door')
    robot.Move(0, 0)

if __name__ == '__main__':
    
    
    # -------------- Start Simulation -------------- #
    client = RemoteAPIClient()
    sim = client.require("sim")
    sim.startSimulation()

    # Initalize the robot:
    robot = Hexa4R("HEXA4R", client=client)
    lidar_sensor = SickTIM310('./SickTIM310',client=client)

    find_door()

    time.sleep(5)
    sim.stopSimulation()
    # Algorithm:
    # 1. Find the corner to detect the exit
    # 2. Walk to the door
    
    