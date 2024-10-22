from coppeliasim_zmqremoteapi_client import *
from MobileRobot import MobileRobot
import math
import time
import csv
import threading

client = RemoteAPIClient()
sim = client.require('sim')
sim.startSimulation()

# Initialize the robot instance
robot = MobileRobot(client, 'HEXA4S')

isRecoding = False
gDistances = []
room = 'room3'

#
def ReadUltrasonicSensor():
    while isRecoding:
        gDistances = robot.ReadUltrasonicSensors()
        print(gDistances)
        time.sleep(0.2)
    
    # Save the ulrasonic sensor data to CSV file
    #with open(f'{room}.csv', mode='w', newline='') as file:
    #    writer = csv.writer(file)
    #    writer.writerow(['Sensor1', 'Sensor2', 'Sensor3', 'Sensor4', 'Sensor5', 'Sensor6', 'Sensor7', 'Sensor8', 'Sensor9', 'Sensor10', 'Sensor11', 'Sensor12'])
    #    while isRecoding:
    #        gDistances = robot.ReadUltrasonicSensors()
    #        writer.writerow(gDistances)
    #        time.sleep(0.2)

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


# Start the thread to read the ultrasonic sensor
isRecoding = True
thread = threading.Thread(target=ReadUltrasonicSensor, args='')
thread.start()

time.sleep(5)

#RotateAlignTheWall('right')

# Stop the thread
isRecoding = False
thread.join()

sim.stopSimulation()

