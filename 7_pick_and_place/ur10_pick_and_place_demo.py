import os
import sys
sys.path.append(os.getcwd())

from coppeliasim_zmqremoteapi_client import *
from lib.ArmRobot import UniversalRobot, Gripper
import time


def main():
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.startSimulation()

    armRobot = UniversalRobot('UR10')
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
    main()

