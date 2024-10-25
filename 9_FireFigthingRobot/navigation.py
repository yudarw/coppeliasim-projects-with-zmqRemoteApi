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
def scan(side, dist, speed=80):
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

    robot.Move(speed + pid_rot + pid_trans, speed - pid_rot - pid_trans)

    # Check the front sensor, if the front sensor is too close rotate robot to the left
    #if dist[0] < 13:
    #    robot.Rotate(-90, 90)
    #    WallAlign(side=side)
    
    # Check the front side sensor to detect an intersection
    # It will make the robot smoothly turn to the right.
    if dist[2] > 80:
        robot.Move(80, 20)
        sim.wait(5)

    #time.sleep(Ts)

# Align the body of the robot with the wall
def WallAlign(side):

    # Search for the closest wall
    while True:
        dist = robot.ReadUltrasonicSensors()
        side_dist = 0
        if side == 'right':
            side_dist = dist[3]
        elif side == 'left':
            side_dist = dist[9]

        if side_dist > 30:
            robot.Move(-90,90)
        else:
            break
        
    # Then rotate align the wall
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


# ----------------------------------------------------------------------------- #
# Algorithm to exit the room
# ----------------------------------------------------------------------------- #
def room_exit(room):

    if room == 'ROOM3' or room == 'ROOM4':        
        while True:
            dist = robot.ReadUltrasonicSensors()
            if dist[0] < 20:
                break
            else:
                scan('right',dist,speed=360)
            time.sleep(0.1)
    
    elif room == 'ROOM2A':
        robot.Rotate(-45, 90)
        #robot.Move(90, 90)
        while True:
            dist = robot.ReadUltrasonicSensors()
            if (dist[0] < 20):
                break
            else:
                robot.Move(150, 150)
            time.sleep(0.1)
        
        robot.Move(0, 0)
        WallAlign('right')

        while True:
            dist = robot.ReadUltrasonicSensors()
            scan ('right', dist)
            if dist[9] > 200:
                time.sleep(1)
                break
            time.sleep(0.1)

    elif room == 'ROOM2B':
        while True:
            dist = robot.ReadUltrasonicSensors()
            scan ('right', dist)
            if dist[9] > 200:
                time.sleep(1)
                break
            time.sleep(0.1)
    robot.Move(0, 0)
    print('Robot has exit the room!')

# --------------------------------------------------------------------------- #
# Algorithm for room detection, we utilize the different of the room size
# to identify the room number.
# --------------------------------------------------------------------------- #
def room_detection():
    room='Unknown'
    # First align the robot to the closest wall
    WallAlign('right')
    # Then we looking for the corner position to detect the room
    while True:
        distance = robot.ReadUltrasonicSensors()
        scan('right', distance)

        # Detect the corner to start the room scanning:
        if distance[0] < 13:
            robot.Rotate(-90, 90)
            WallAlign('right')
            distance = robot.ReadUltrasonicSensors()
            lidar = lidar_sensor.getAllDistances()
            lidar_front = np.mean(lidar[130:140])
            lidar_diagonal = np.mean(lidar[175:185])
            lidar_left = distance[9]
            lidar_left_mean = np.mean(lidar[180:269])
            room=''
            if lidar_front > 80:

                # ROOM4:
                if (lidar_left > 55 and lidar_left < 75) and (lidar_front > 90 and lidar_front < 120) and (lidar_left_mean > 45 and lidar_left_mean < 65):
                    room = 'ROOM4'
                elif (lidar_left > 55 and lidar_left < 80) and (lidar_front > 90 and lidar_front < 120) and (lidar_left_mean > 65 and lidar_left_mean < 85):
                    room = "ROOM2A"
                elif (lidar_left > 90 and lidar_left < 110) and (lidar_front > 130 and lidar_front < 300) and (lidar_left_mean > 60 and lidar_left_mean < 80):
                    room = "ROOM2B"
                elif (lidar_left > 60 and lidar_left < 85) and (lidar_front > 130  and lidar_front < 160) and (lidar_left_mean > 60 and lidar_left_mean < 80):
                    room = "ROOM3"
                elif (lidar_left > 200) and (lidar_front > 250 and lidar_front < 300):
                    room = "ROOM1A"
                elif (lidar_left > 110 and lidar_left < 130) and (lidar_front > 80 and lidar_front < 100):
                    room = "ROOM1B"

                print(f'L={lidar_left}, F={lidar_front}, L_mean={lidar_left_mean}, ROOM={room}')
                break
    
    print('Room detection completed!')
    robot.Move(0, 0)
    return room


if __name__ == '__main__':

    # -------------- Start Simulation -------------- #
    client = RemoteAPIClient()
    sim = client.require("sim")
    sim.startSimulation()

    # Initalize the robot:
    robot = Hexa4R("HEXA4R", client=client)
    lidar_sensor = SickTIM310('./SickTIM310',client=client)

    room=room_detection()
    room_exit(room)

    time.sleep(5)
    sim.stopSimulation()
    # Algorithm:
    # 1. Find the corner to detect the exit
    # 2. Walk to the door
    
    