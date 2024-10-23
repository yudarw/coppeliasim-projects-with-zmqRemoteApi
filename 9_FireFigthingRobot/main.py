from coppeliasim_zmqremoteapi_client import *
import math
import time
from MobileRobot import MobileRobot

client = RemoteAPIClient()
sim = client.require('sim')
sim.startSimulation()

# initialize the robot instance
robot = MobileRobot(client, 'HEXA4R')

# ----------------  Implement the Wall Following Algorithm ---------------- #
def ScanWall(side):

    # parameters for pid rotation
    Kp_rot = 2
    Ki_rot = 0.001   
    Kd_rot = 0.5
    error_rot = 0
    last_error_rot = 0
    pid_rot = 0
    sum_error_rot = 0

    # parameters for pid translation
    Kp_trans = 2
    Ki_tans = 0
    Kd_tans = 0
    error_trans = 0
    last_error_trans = 0
    pid_trans = 0

    wall_distance = 13      # 5 cm from the wall
    Ts = 0.05               # Time sampling 
    Sp = 200                # Initial speed

    while True:
        
        # Read distance
        dist = robot.ReadUltrasonicSensors()
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

        print(f'error_rot: {error_rot}, error_trans: {error_trans}')
        #print(f'dist from wall: {side_dist} cm')    

        robot.Move(Sp + pid_rot + pid_trans, Sp - pid_rot - pid_trans)

        # Check the front sensor, if the front sensor is too close rotate robot to the left
        if dist[0] < 15:
            robot.Rotate(-90, 90)

        # Check the front side sensor to detect an intersection
        # It will make the robot smoothly turn to the right.
        if dist[2] > 80:
            robot.Move(140, 40)
            time.sleep(0.9)

        time.sleep(Ts)


# Align the body of the robot with the wall
def SetWallParallel(side):
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
        
        P = 3 * error
        robot.Move(P, -P)
        
        if abs(error) < 2:
            break
        
        time.sleep(0.05)
    robot.Move(0, 0)


# --------------- Test --------------------- #
SetWallParallel('right')
ScanWall('right')

time.sleep(3)
sim.stopSimulation()