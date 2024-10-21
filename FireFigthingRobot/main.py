from coppeliasim_zmqremoteapi_client import *
import math
import time

from MobileRobot import MobileRobot

client = RemoteAPIClient()
sim = client.require('sim')
sim.startSimulation()

robot = MobileRobot(client, 'HEXA4S')


def ScanWall(side):
    Kp_rot = 2
    Ki_rot = 0
    Kd_rot = 0.9
    error_rot = 0
    last_error_rot = 0
    pid_rot = 0
    sum_error_rot = 0

    Kp_trans = 0
    Ki_tans = 0
    Kd_tans = 0
    error_trans = 0
    last_error_trans = 0
    pid_trans = 0

    Ts = 0.05
    Sp = 90

    while True:

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
        
        error_rot = side_front_dist - side_back_dist
        if error_rot > 40:
            error_rot = 40
        
        if error_rot < -40:
            error_rot = -40

        last_error_rot = error_rot
        sum_error_rot += error_rot

        pid_rot = Kp_rot * error_rot + (Kd_rot * (error_rot - last_error_rot) / Ts) + (Ki_rot * sum_error_rot * Ts)
        robot.Move(Sp + pid_rot, Sp - pid_rot)

        # Check front sensor
        if dist[0] < 10:
            robot.Rotate2(-89, 90)

        time.sleep(Ts)

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
        print(error)
        
        if error > 50:
            error = 50
        
        P = 3 * error
        robot.Move(P, -P)
        
        if abs(error) < 2:
            break
        
        time.sleep(0.05)
    robot.Move(0, 0)

def Demo():
    pass

SetWallParallel('right')
ScanWall('right')

time.sleep(3)
sim.stopSimulation()