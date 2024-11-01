import os
import sys
sys.path.append(os.getcwd())

from coppeliasim_zmqremoteapi_client import *
from lib.ArmRobot import UniversalRobot, Gripper
import time
import csv

client = RemoteAPIClient()
sim = client.require('sim')
armRobot = UniversalRobot('UR10')

def SavePalletPositiion():
    with open('pallet_positions.csv', mode='w') as file:
        writer = csv.writer(file)
        for i in range(21):
            pos = armRobot.GetObjectPosition(f'Cartoons{i + 1}')
            print(f"position of Cartoons{i + 1}: {pos}")
            writer.writerow(pos)

def main():
    sim.startSimulation()

    
    armRobot.AttachGripper('vacuum_gripper')
    armRobot.gripper.Catch()
    
    # Sensor handle:
    prox_sensor = sim.getObject('/ConveyorSensor')

    while True:
        ret = sim.readProximitySensor(prox_sensor)
        if ret[0] == 1:

            # PickUp Position
            pickUpPos = [570, 140, 90, 180, 0, 90]
            armRobot.MoveL(pickUpPos,50)

            time.sleep(0.5)

            liftUpPos = [570, 140, 200, 180, 0, 90]
            armRobot.MoveL(liftUpPos,50)

            time.sleep(1)

            armRobot.gripper.Release()

            time.sleep(1)
            break

        time.sleep(0.1)

    sim.stopSimulation()

if __name__ == '__main__':
    #main()
    sim.startSimulation()
    SavePalletPositiion()

