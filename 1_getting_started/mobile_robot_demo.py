from coppeliasim_zmqremoteapi_client import *
import time
import math
import numpy as np

DEG_TO_RAD = math.pi / 180

client = RemoteAPIClient()
sim = client.require('sim')
sim.startSimulation()

# Get the handle of left and right motors
motorLeft = sim.getObject('./leftMotor')
motorRight = sim.getObject('./rightMotor')
print('motorLeft handle :', motorLeft)
print('motorRight handle :', motorRight)

# Set the robot speed
 # Initialize the robot
print('Robot move forward')
sim.setJointTargetVelocity(motorLeft, 30 * DEG_TO_RAD)
sim.setJointTargetVelocity(motorRight, 30 * DEG_TO_RAD)
time.sleep(3)

print('Robot move backward')
sim.setJointTargetVelocity(motorLeft, -30 * DEG_TO_RAD)
sim.setJointTargetVelocity(motorRight, -30 * DEG_TO_RAD)
time.sleep(3)

print('Robot Rotate Left')
sim.setJointTargetVelocity(motorLeft, -30 * DEG_TO_RAD)
sim.setJointTargetVelocity(motorRight, 30 * DEG_TO_RAD)
time.sleep(3)

print('Robot Rotate Right')
sim.setJointTargetVelocity(motorLeft, 30 * DEG_TO_RAD)
sim.setJointTargetVelocity(motorRight, -30 * DEG_TO_RAD)
time.sleep(3)

print('Robot Stop')
sim.setJointTargetVelocity(motorLeft, 0)
sim.setJointTargetVelocity(motorRight, 0)

sim.stopSimulation()