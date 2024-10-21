from coppeliasim_zmqremoteapi_client import *
import time
import math
import numpy as np

DEG_TO_RAD = math.pi / 180

client = RemoteAPIClient()
sim = client.require('sim')
sim.startSimulation()

# Get the handle of left and right motors
motorLeft1 = sim.getObject('./wheel1')
motorLeft2 = sim.getObject('./wheel4')
motorRight1 = sim.getObject('./wheel2')
motorRight2 = sim.getObject('./wheel3')
print('motorLeft handle :', motorLeft1)
print('motorRight handle :', motorRight1)

# Set the robot speed
 # Initialize the robot
print('Robot move forward')
sim.setJointTargetVelocity(motorLeft1, 30 * DEG_TO_RAD)
sim.setJointTargetVelocity(motorLeft2, 30 * DEG_TO_RAD)
sim.setJointTargetVelocity(motorRight1, 30 * DEG_TO_RAD)
sim.setJointTargetVelocity(motorRight2, 30 * DEG_TO_RAD)
time.sleep(3)

# print('Robot move backward')
sim.setJointTargetVelocity(motorLeft1, 20 * DEG_TO_RAD)
sim.setJointTargetVelocity(motorLeft2, 20 * DEG_TO_RAD)
sim.setJointTargetVelocity(motorRight1, -20 * DEG_TO_RAD)
sim.setJointTargetVelocity(motorRight2, -20 * DEG_TO_RAD)
time.sleep(0.5)

# print('Robot move backward')
sim.setJointTargetVelocity(motorLeft1, 30 * DEG_TO_RAD)
sim.setJointTargetVelocity(motorLeft2, 30 * DEG_TO_RAD)
sim.setJointTargetVelocity(motorRight1, 30 * DEG_TO_RAD)
sim.setJointTargetVelocity(motorRight2, 30 * DEG_TO_RAD)
time.sleep(3)

# print('Robot Rotate Right')
# sim.setJointTargetVelocity(motorLeft, 30 * DEG_TO_RAD)
# sim.setJointTargetVelocity(motorRight, -30 * DEG_TO_RAD)
# time.sleep(3)

# print('Robot Stop')
# sim.setJointTargetVelocity(motorLeft, 0)
# sim.setJointTargetVelocity(motorRight, 0)

sim.stopSimulation()