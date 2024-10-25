from coppeliasim_zmqremoteapi_client import *
import threading
import time
import struct
import matplotlib.pyplot as plt
import numpy as np

class SickTIM310:

    def __init__(self,sensor_name,client=None):
        try:
            
            if client == None:
                client = RemoteAPIClient()
            
            self.sim = client.require('sim')
            # Get the sensor and script handle
            self.sensor = self.sim.getObject(f'{sensor_name}')
            self.script_handle = self.sim.getScript(self.sim.scripttype_childscript,self.sensor)
            self.lidar_data = []

        except Exception as e:
            print('Error: ', e)
            self.client.__del__()    

    # Get the 2D mapping of the liDar sensor
    def get2DMapping(self):       
        self.lidar_data = self.sim.callScriptFunction('get_data', self.script_handle)
        x = [self.lidar_data[3 * i] for i in range(270)]
        y = [self.lidar_data[3 * i + 1] for i in range(270)]
        return x, y

    # Plot the 2D mapping of the image
    def plot2DMapping(self):
        x, y = self.get2DMapping()
        plt.scatter(x, y)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('2D Mapping of LiDAR Sensor Data')
        #plt.savefig('lidar_2d_mapping.png')
        plt.show()

    # Angle is map from -45 to 225
    def  getDistance(self, angle):
        # Make sure the angle is in the range of -45 to 225
        if angle < -45 or angle > 225:
            print('Error: Angle out of range')
            return None

        normalized_angle = angle + 45

        # Get the data from the sensor:
        self.lidar_data = self.sim.callScriptFunction('get_data', self.script_handle)
        x = self.lidar_data[3 * normalized_angle]
        y = self.lidar_data[3 * normalized_angle + 1]
        
        # Calulate the norm of the vector to get the distance:
        distance = np.linalg.norm([x, y]) * 100
        return distance
    
    def getAllDistances(self):
        distances = []
        self.lidar_data = self.sim.callScriptFunction('get_data', self.script_handle)
        for i in range(270):
            x = self.lidar_data[3 * i]
            y = self.lidar_data[3 * i + 1]
            distance = np.linalg.norm([x, y]) * 100
            distances.append(distance)
            #print('Angle: ', i, 'Distance: ', distance)
        return distances


if __name__ == '__main__':
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.startSimulation()

    sensor = SickTIM310('./SickTIM310')
    sensor.getAllDistances()
    sensor.plot2DMapping()


















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