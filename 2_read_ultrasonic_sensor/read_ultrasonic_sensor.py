from coppeliasim_zmqremoteapi_client import *
import time
import math
import numpy as np

# Attempt to connect to the Coppeliasim
client = RemoteAPIClient()
sim = client.require('sim')

# Start Simulation:
sim.startSimulation()

# Retrieve ultrasonic sensor handles
sensorHandles = np.zeros(16)
for i in range(16):
    sensorHandles[i] = sim.getObject(f'./ultrasonicSensor[{i}]')
print('sensor handles :', sensorHandles)


# Read sensor:
while True:
    for i in range(16):
        state, dist, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.readProximitySensor(sensorHandles[i])
        if state:
            print(f'Ultrasonic sensor {i} detected object {detectedObjectHandle} at distance {dist}')
        else:
            print(f'Ultrasonic sensor {i} detected nothing')

    time.sleep(1)