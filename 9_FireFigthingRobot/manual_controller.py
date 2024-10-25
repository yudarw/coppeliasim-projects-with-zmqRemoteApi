import sys
import os
sys.path.append(os.getcwd())

from coppeliasim_zmqremoteapi_client import *
import keyboard
import time
from lib.MobileRobot import Hexa4R
from lib.sick_tim310 import SickTIM310
import threading
import numpy as np

robot = None
sensor = None
stop_sensing_thread_event = threading.Event()
distances = None

def read_lidar():
    distances = sensor.getAllDistances()
    
    # get the average value in left, right, front, and back
    right_dist = np.mean(distances[40:50])
    left_dist = np.mean(distances[220:230])
    front_dist = np.mean(distances[130:140])

    room_size = front_dist * left_dist
    print(f'Length={np.round(front_dist)}, Width={np.round(left_dist)}, Size={np.round(room_size)}')

    #sum_of_distance = 0
    #for i in range(len(distances)):
    #    sum_of_distance = sum_of_distance + distances[i]
    #print(f"Sum of distances: {sum_of_distance}")
    #print('Exit thread sensing...')
    

if __name__ == '__main__':
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.startSimulation()
    robot = Hexa4R('HEXA4R',client=client)
    sensor = SickTIM310('./SickTIM310',client=client)
    robot.Move(0, 0)

    # start sensing thread
    #thread_sensing = threading.Thread(target=read_lidar)
    #thread_sensing.start()

    print('Press up, down, left, and right arrow to navigate the robot...')
    print('Press ESC to exit..')
    while True:
        read_lidar()

        if keyboard.is_pressed('up'):
            robot.Move(90, 90)
            time.sleep(0.1)
        
        elif keyboard.is_pressed('down'):
            robot.Move(-90, -90)
            time.sleep(0.1)

        elif keyboard.is_pressed('left'):
            robot.Move(-90, 90)
            time.sleep(0.1)

        elif keyboard.is_pressed('right'):
            robot.Move(90, -90)
            time.sleep(0.1)
    
        elif keyboard.is_pressed('esc'):
            break

        else:
            robot.Move(0, 0)
            time.sleep(0.1)
    
    #stop_sensing_thread_event.set()
    #thread_sensing.join()

    sim.stopSimulation()
    print('Program end!')