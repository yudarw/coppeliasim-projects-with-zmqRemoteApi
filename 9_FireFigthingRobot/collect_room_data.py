from coppeliasim_zmqremoteapi_client import *
import math
import time
from MobileRobot import MobileRobot

client = RemoteAPIClient()
sim = client.require('sim')
sim.startSimulation()

# initialize the robot instance
robot = MobileRobot(client, 'HEXA4S')