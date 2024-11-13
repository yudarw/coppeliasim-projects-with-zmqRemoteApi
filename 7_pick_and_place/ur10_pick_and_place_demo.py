import os
import sys
sys.path.append(os.getcwd())

from coppeliasim_zmqremoteapi_client import *
from lib.ArmRobot import UniversalRobot, Gripper
import time
import csv

# Initialize Coppeliasim Remote API Client
client = RemoteAPIClient()
sim = client.require('sim')

# Initialize the UR10 robot and attach the vacuum gripper
armRobot = UniversalRobot('UR10')
armRobot.AttachGripper('vacuum_gripper')


# Automatically Record the pallet positions
# ------------------------------------------------------------------------ #
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


# Load the pallet positions from the CSV file
# ------------------------------------------------------------------------ #
def LoadPalletPosition():
    positions = []
    with open('pallet_positions.csv', mode='r') as file:
        reader = csv.reader(file)
        for row in reader:
            positions.append([float(i) for i in row])
    return positions


# Put the object to the pallet
# ------------------------------------------------------------------------ #
def putObjectToPallet(count, palletPos):
    # First, lift up the object from the conveyor
    pos = [570, 0, palletPos[2] + 400, 180, 0, palletPos[5] - 90]
    if count > 13:
        pos = [570, 0, palletPos[2] + 100, 180, 0, palletPos[5] - 90]
    armRobot.MoveL(pos, 300)

    # Waypoint to avoid the singularity
    #pos = [570, -400, palletPos[2] + 400, 180, 0, palletPos[5] - 90]
    #armRobot.MoveL(pos, 500)

    if count == 14:
        set_hydrolic_position(200)

    # Next, move the workpiece to the target position
    # -------------------------------------------------------------------- #
    target_pos_up   = [palletPos[0], palletPos[1], palletPos[2] + 200, 180, 0, palletPos[5] - 90] 
    target_pos_down = [palletPos[0], palletPos[1], palletPos[2] + 110, 180, 0, palletPos[5] - 90]
    
    # For the third layer we decrease the height 20cm lower
    if count > 13:
        target_pos_up   = [palletPos[0], palletPos[1], palletPos[2] + 150 - 200, 180, 0, palletPos[5] - 90] 
        target_pos_down = [palletPos[0], palletPos[1], palletPos[2] + 110 - 200, 180, 0, palletPos[5] - 90]
    
    armRobot.MoveL(target_pos_up, 500)
    # Print joint position:
    jointPos = armRobot.ReadJointPosition()
    print(f'Joint position [{count}]: {jointPos}')

    armRobot.MoveL(target_pos_down, 50)
    armRobot.gripper.Release()

    armRobot.MoveL(target_pos_up, 500)


    # Last, move the robot back to the pickup position
    # -------------------------------------------------------------------- #
    #waypoint2 = [400, -500, 400, 180, 0, 90]
    #armRobot.MoveL(waypoint2, 800)

    standbyPos = [700, 0, -80, 180, 0, 90]
    jointpos = [-14.211824351788398, 23.613074715029327, 122.55519954862501, -56.18447177751439, -90.09695623063273, 75.74877825490448]
    if count > 13:
        jointpos = [-14.211824351788398, 23.613074715029327, 122.55519954862501, -56.18447177751439, -90.09695623063273, 75.74877825490448]
        standbyPos = [700, 0, -200, 180, 0, 90]
    
    #armRobot.MoveL(standbyPos, 800)
    # Print joint position:
    #jointPos = armRobot.ReadJointPosition()
    #print(f'StandbyPos [{count}]: {jointPos}')

    # -------------------------------------------------------------------- #

    # Move using joint mode:
    #jointpos = [-14.208260756479737, 35.46053907427984, 123.15032846861341, -68.62691452867405, -90.09669869797803, 75.74844865863369]
    armRobot.MoveJ(jointpos, 180)

    
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


# ------------------------------------------------------------------------ #
# Main function
# ------------------------------------------------------------------------ #
def main2():
    sim.startSimulation()

    # Get proximity sensor handle to detect the incoming object
    prox_sensor = sim.getObject('/ConveyorSensor')

    # Load the target positions from the CSV
    targetPositions = LoadPalletPosition()
    for i in range(len(targetPositions)):
        print(f'Box position [{i}]: {targetPositions[i]}')
    
    standbyPos = [570, 0, 400, 180, 0, 90]
    armRobot.MoveL(standbyPos, 100)

    # Setup the pickup position of the object in the conveyor
    pickUpPos = [570, 0, 87, 180, 0, 90]
    liftUpPos = [570, 0, 200, 180, 0, 90]

    # Complete the pallet collection
    # -------------------------------------------------------------------- #
    count = 0
    while count < 21:
        ret,dist,pos,handle,norm = sim.readProximitySensor(prox_sensor)

        # If the object is detected, pick it up
        if ret == 1:
            print('New object detected...')

            # Wait for the workpiece to completely stop
            time.sleep(0.2)

            # Read the position of the detected object
            box_position = armRobot.GetObjectPosition2(handle)
            
            # Move the robot to the pickup position
            print('Moving to the pickup position...')
            pickUpPos = [box_position[0], box_position[1], box_position[2] + 105, 180, 0, 90]
            armRobot.MoveL(pickUpPos, 300)
            armRobot.gripper.Catch()

            # Put the workpiece to the pallet
            print('Moving to the target position...')
            putObjectToPallet(count, targetPositions[count])
            
            count = count + 1
            print(f'Object {count} has been placed to the pallet...\n\n')

        time.sleep(0.1)

    # Stop the simulation
    time.sleep(5)
    sim.stopSimulation()


def main():
    sim.startSimulation()
    speed = 100
    pos1 = [570, 0, 87, 180, 0, 90]
    pos2 = [570, 0, 200, 180, 0, 90]
    armRobot.MoveL(pos1, speed)
    armRobot.MoveL(pos2, speed)


if __name__ == '__main__':
    main()
    #record_positions()
    #ur10_hydrolic_test()
    


