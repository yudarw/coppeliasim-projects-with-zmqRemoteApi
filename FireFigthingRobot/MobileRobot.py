from coppeliasim_zmqremoteapi_client import *
import math
import time


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
        # Get vision sensor handle:
        self.camera_sensor = self.sim.getObject(f'/{robotName}/camera')

    def Move(self, leftVel, rightVel):
        self.sim.setJointTargetVelocity(self.motorL1, leftVel * math.pi/180)
        self.sim.setJointTargetVelocity(self.motorL2, leftVel * math.pi/180)
        self.sim.setJointTargetVelocity(self.motorR1, rightVel* math.pi/180)
        self.sim.setJointTargetVelocity(self.motorR2, rightVel* math.pi/180)

    def Rotate(self, tetha, speed):
        initialOrientation = 0
        ori = self.sim.getObjectOrientation(self.simrobot, -1)[2] * 180 / math.pi
        
        if ori < 0:
            initialOrientation = (360 + ori) % 360 
        else:
            initialOrientation = ori

        targetOrientation = initialOrientation - tetha
        if (targetOrientation > 360):
            targetOrientation = targetOrientation - 360

        # ------------------- Rotate Left ---------------- #
        if tetha < 0:
            print("Rotate Left")
            print("Ori: ", ori)
            print("InitialOrientation: ", initialOrientation)
            print("TargetOrientation: ", targetOrientation)
            self.Move(-speed, speed)
            currentOrientation = initialOrientation
            while targetOrientation > currentOrientation:
                currentOrientation = self.sim.getObjectOrientation(self.simrobot, -1)[2] * 180 / math.pi
                if currentOrientation < 0:
                    currentOrientation = 360 + currentOrientation
                time.sleep(0.05)

        
        self.Move(0, 0)

    def Rotate2(self, tetha, speed):
        initOri = self.sim.getObjectOrientation(self.simrobot, -1)[2] * 180 / math.pi
        targetOri = initOri - tetha
        
        if targetOri > 180:
                targetOri = (targetOri - 180) - 180
        elif targetOri < -180:
            targetOri = (targetOri + 180) + 180  

        print(initOri)
        print(targetOri)

        if tetha < 0:
            self.Move(-speed, speed)
        elif tetha > 0:
            self.Move(speed, -speed)

        while True:
            currentOri = self.sim.getObjectOrientation(self.simrobot, -1)[2] * 180 / math.pi
            if abs(currentOri - targetOri) < 2.0:
                break

        self.Move(0, 0) 



    def ReadUltrasonicSensors(self):
        distance = []
        for i in range(12):
            ret = self.sim.readProximitySensor(self.ultrasonic_sensor[i])
            if(ret[1] == 0):
                ret[1] = 3.00;           
            distance.append(ret[1] * 100) # convert to mm
        return distance
    


if __name__ == '__main__':
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.startSimulation()

    robot = MobileRobot(client, 'HEXA4S')
    
    robot.Rotate2(-30, 90)
    #robot.Move(-10,10)
    

    #distance = robot.ReadUltrasonicSensors()
    #print(distance)


    time.sleep(3)
    sim.stopSimulation()
