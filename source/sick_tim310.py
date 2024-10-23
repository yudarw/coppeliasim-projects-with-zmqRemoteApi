import sys
import os
sys.path.append(os.getcwd())
from coppeliasim_zmqremoteapi_client import *
from lib.MobileRobot import MobileRobot
import threading
import time
import struct
import matplotlib.pyplot as plt
import numpy as np


class SickTIM310:

    def __init__(self, sensor_name):
        client = RemoteAPIClient()
        self.sim = client.require('sim')
        # Get the sensor and script handle
        self.sensor = self.sim.getObject(f'{sensor_name}')
        self.script_handle = self.sim.getScript('')














if __name__ == '__main__':
    pass


















# client = RemoteAPIClient()
# sim = client.require('sim')
# sim.startSimulation()

# time.sleep(1)

# h = sim.getObject('./SickTIM310')
# script_handle = sim.getScript(sim.scripttype_childscript,h)
# data = sim.callScriptFunction('get_data', script_handle)

# x = []
# y = []
# for i in range(270):
#     x.append(data[3 * i])
#     y.append(data[3 * i + 1])

# plt.figure(figsize=(8, 8))
# plt.scatter(x, y, c='blue', marker=0, s=10)
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.title('Sick TIM310 Data')
# plt.axis('equal')
# plt.grid(True)
# plt.show()