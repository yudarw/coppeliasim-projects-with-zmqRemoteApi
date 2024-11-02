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
        height = 0
        for i in range(21):
            if i != 0 and i % 7 == 0:
                height = height + 200
                #set_hydrolic_position(height)
            pos = armRobot.GetObjectPosition(f'Cartoons{i + 1}')
            print(pos)
            writer.writerow(pos)
            time.sleep(0.2)

def LoadPalletPosition():
    positions = []
    with open('pallet_positions.csv', mode='r') as file:
        reader = csv.reader(file)
        for row in reader:
            positions.append([float(i) for i in row])
    return positions

def putObjectToPallet(count, palletPos):

    # 1. Lift up from conveyor
    pos = [570, 0, palletPos[2] + 400, 180, 0, palletPos[5] - 90]
    if count > 13:
        pos = [570, 0, palletPos[2] + 100, 180, 0, palletPos[5] - 90]
    
    armRobot.MoveL(pos, 300)
    pos = [570, -400, palletPos[2] + 400, 180, 0, palletPos[5] - 90]
    #armRobot.MoveL(pos, 500)

    if count == 14:
                set_hydrolic_position(200)

    # Move to the pallet position
    target_pos_up = [palletPos[0], palletPos[1], palletPos[2] + 200, 180, 0, palletPos[5] - 90] 
    target_pos_down = [palletPos[0], palletPos[1], palletPos[2] + 110, 180, 0, palletPos[5] - 90]
    
    if count > 13:
        target_pos_up = [palletPos[0], palletPos[1], palletPos[2] + 150 - 200, 180, 0, palletPos[5] - 90] 
        target_pos_down = [palletPos[0], palletPos[1], palletPos[2] + 110 - 200, 180, 0, palletPos[5] - 90]
    
    
    #print(f"Move to pallet position: {pos}")
    armRobot.MoveL(target_pos_up, 500)
    armRobot.MoveL(target_pos_down, 50)
    armRobot.gripper.Release()
    armRobot.MoveL(target_pos_up, 500)

    standbyPos2 = [400, -500, 400, 180, 0, 90]
    #armRobot.MoveL(standbyPos2, 800)

    standbyPos = [700, 0, -80, 180, 0, 90]
    if count > 13:
        standbyPos = [700, 0, -200, 180, 0, 90]
    
    armRobot.MoveL(standbyPos, 800)
    #time.sleep(1)

def main():
    sim.startSimulation()
    # Get proximity sensor handle to detect the incoming object
    prox_sensor = sim.getObject('/ConveyorSensor')

    # Load the target positions
    targetPositions = LoadPalletPosition()
    for pos in targetPositions:
        print(pos)
    
    standbyPos = [570, 0, 400, 180, 0, 90]
    armRobot.MoveL(standbyPos, 100)

    # Setup the pickup position of the object in the conveyor
    pickUpPos = [570, 0, 87, 180, 0, 90]
    liftUpPos = [570, 0, 200, 180, 0, 90]

    # Complete the pallet collection
    count = 0
    while count < 21:
        ret,dist,pos,handle,norm = sim.readProximitySensor(prox_sensor)
        if ret == 1:
            time.sleep(0.2)
            objPos = armRobot.GetObjectPosition2(handle)
            pickUpPos = [objPos[0], objPos[1], objPos[2] + 105, 180, 0, 90]
            armRobot.MoveL(pickUpPos, 300)
            armRobot.gripper.Catch()

            putObjectToPallet(count, targetPositions[count])
            count = count + 1
        time.sleep(0.1)

    time.sleep(5)
    sim.stopSimulation()


def record_positions():
    sim.startSimulation()
    SavePalletPosition()
    targetPositions = LoadPalletPosition()
    for pos in targetPositions:
       print(pos)


def set_hydrolic_position(height):
    sim.startSimulation()
    slider = sim.getObject('/ur10_hydrolic/slider')    
    sim.setJointTargetPosition(slider, height/1000)
    sim.wait(10)

if __name__ == '__main__':
    main()
    #record_positions()
    #ur10_hydrolic_test()
    


