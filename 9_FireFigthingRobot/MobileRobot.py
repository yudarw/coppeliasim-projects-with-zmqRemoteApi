from coppeliasim_zmqremoteapi_client import *
import math
import time
import threading

class MobileRobot:

    def __init__(self, client, robotName) -> None:
        self.sim = client.require('sim')
        # Get robot and motors handle
        self.simrobot = self.sim.getObject(f'/{robotName}')
        self.motorL1 = self.sim.getObject(f'/{robotName}/motor_1')
        self.motorL2 = self.sim.getObject(f'/{robotName}/motor_2')
        self.motorR1 = self.sim.getObject(f'/{robotName}/motor_3')
        self.motorR2 = self.sim.getObject(f'/{robotName}/motor_4')        
        # Get sensor handle:
        self.ultrasonic_sensor = [self.sim.getObject(f'/{robotName}/ultrasonic_sensor{i + 1}') for i in range (12)]
        self.ultrasonic_data = [0] * 12
        # Get vision sensor handle:
        self.camera_sensor = self.sim.getObject(f'/{robotName}/camera')
        self.stop_thread_event = threading.Event()

    def Move(self, leftVel, rightVel):
        self.sim.setJointTargetVelocity(self.motorL1, leftVel * math.pi/180)
        self.sim.setJointTargetVelocity(self.motorL2, leftVel * math.pi/180)
        self.sim.setJointTargetVelocity(self.motorR1, rightVel* math.pi/180)
        self.sim.setJointTargetVelocity(self.motorR2, rightVel* math.pi/180)

    def Rotate(self, tetha, speed):
        initOri = self.sim.getObjectOrientation(self.simrobot, -1)[2] * 180 / math.pi
        targetOri = initOri - tetha
        
        if targetOri > 180:
                targetOri = (targetOri - 180) - 180
        elif targetOri < -180:
            targetOri = (targetOri + 180) + 180  

        if tetha < 0:
            self.Move(-speed, speed)
        elif tetha > 0:
            self.Move(speed, -speed)

        while True:
            currentOri = self.sim.getObjectOrientation(self.simrobot, -1)[2] * 180 / math.pi
            if abs(currentOri - targetOri) < 2.0:
                break
            time.sleep(0.05)
        self.Move(0, 0) 

    def ReadUltrasonicSensors(self):
        distance = []
        for i in range(12):
            ret = self.sim.readProximitySensor(self.ultrasonic_sensor[i])
            dist = ret[1]
            if dist == 0:
                dust = 3.00;           
            distance.append(dist * 100) # convert to cm
        return distance
    
    def enable_sensing(self):
        self.stop_thread_event.clear()
        self.ultrasonic_thread = threading.Thread(target=self.thread_read_ultrasonic_sensor)
        self.ultrasonic_thread.start()

    def disable_sensing(self):
        self.stop_thread_event.set()
        self.ultrasonic_thread.join()

    def thread_read_ultrasonic_sensor(self):
        while not self.stop_thread_event.is_set(): 
            self.ultrasonic_data = self.ReadUltrasonicSensors()
            print(self.ultrasonic_data)
            time.sleep(0.1) 


if __name__ == '__main__':
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.startSimulation()

    robot = MobileRobot(client, 'HEXA4S')
    
    #robot.Rotate2(-30, 90)
    #robot.Move(-10,10)

    #distance = robot.ReadUltrasonicSensors()
    #print(distance)

    time.sleep(3)
    sim.stopSimulation()
