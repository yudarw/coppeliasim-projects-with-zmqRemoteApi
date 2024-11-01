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
armRobot.AttachGripper('vacuum_gripper')



def SavePalletPosition():
    with open('pallet_positions.csv', mode='w',newline='') as file:
        writer = csv.writer(file)
        for i in range(21):
            pos = armRobot.GetObjectPosition(f'Cartoons{i + 1}')
            writer.writerow(pos)

def LoadPalletPosition():
    positions = []
    with open('pallet_positions.csv', mode='r') as file:
        reader = csv.reader(file)
        for row in reader:
            positions.append([float(i) for i in row])
    return positions


def main():
    sim.startSimulation()
    # Get proximity sensor handle to detect the incoming object
    prox_sensor = sim.getObject('/ConveyorSensor')

    # Setup the pickup position of the object in the conveyor
    pickUpPos = [570, 140, 87, 180, 0, 90]
    liftUpPos = [570, 140, 200, 180, 0, 90]
    
    # Complete the pallet collection
    count = 0
    while True:
        ret = sim.readProximitySensor(prox_sensor)
        if ret[0] == 1:
            pass
        time.sleep(0.1)

    sim.stopSimulation()

if __name__ == '__main__':
    main()

