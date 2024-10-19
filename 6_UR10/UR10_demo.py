from coppeliasim_zmqremoteapi_client import *
import math
import time
from interpolation import linear_interpolation

client = RemoteAPIClient()
sim = client.require('sim')
sim.startSimulation()


class UR10:
    
    def __init__(self, client, robotName):
        self.sim = client.require('sim')
        self.simIK = client.require('simIK')
        self.robotName = robotName

        # initialize robot arm
        self.simrobot = self.sim.getObject(f'/{robotName}')
        self.simTip = self.sim.getObject(f'/{robotName}/ikTip')
        self.simTarget = self.sim.getObject(f'/{robotName}/ikTarget')

        # Initialize the UR10 inverse kinematic
        self.ikEnv = self.simIK.createEnvironment()
        self.ikGroup = self.simIK.createGroup(self.ikEnv)
        self.simIK.addElementFromScene(self.ikEnv, self.ikGroup, self.simrobot, self.simTip, self.simTarget, self.simIK.constraint_pose)

    # Read the cartesion position of the robot
    def ReadPosition(self):
        pos = self.sim.getObjectPosition(self.simTip, self.simrobot)
        ori = self.sim.getObjectOrientation(self.simTip, self.simrobot)
        pos = [pos[i] * 1000 for i in range(3)]
        ori = [ori[i] * 180 / math.pi for i in range(3)]
        return [pos[0],pos[1],pos[2],ori[0],ori[1],ori[2]]
    
    # Set robot position in mm and deg
    def SetPosition(self, targetPos):
        pos = [targetPos[i] / 1000 for i in range(3)]
        ori = [targetPos[i + 3] * math.pi / 180 for i in range(3)]
        self.sim.setObjectPosition(self.simTarget, pos, self.simrobot)
        self.sim.setObjectOrientation(self.simTarget, ori, self.simrobot)
        # Apply ik
        #self.simIK.handleGroup(self.ikEnv, self.ikGroup, {'syncWorlds' : True, 'allowError' : True})
        self.simIK.applyIkEnvironmentToScene(self.ikEnv, self.ikGroup)
    
    def MoveL(self, targetPos, speed):
        initialPos = self.ReadPosition()
        interpolated_points, steps, time_per_step = linear_interpolation(initialPos, targetPos, speed, speed/20)
        for point in interpolated_points:
            self.SetPosition(point)
            
            if (time_per_step < 0.05):
                time_per_step = 0.05
            time.sleep(time_per_step)
    
    def GetObjectPosition(self, objectName):
        handle = sim.getObject(f'/{objectName}')
        pos = self.sim.getObjectPosition(handle, self.simrobot)
        ori = self.sim.getObjectOrientation(handle, self.simrobot)
        pos = [pos[i] * 1000 for i in range(3)]
        ori = [ori[i] * 180 / math.pi for i in range(3)]
        return [pos[0],pos[1],pos[2],ori[0],ori[1],ori[2]]

    def MoveL_2(self, targetPos, speed):
        pass


if __name__ == '__main__':
        
        armrobot = UR10(client, 'UR3')

        intialpos = armrobot.ReadPosition()
        targetPos = armrobot.GetObjectPosition('target1')
        targetPos2 = armrobot.GetObjectPosition('target2')
        print(f'initial position: {intialpos}')
        print(f'target position: {targetPos}')
        
        while True:
            armrobot.MoveL(targetPos, 300)
            armrobot.MoveL(targetPos2, 200)
            armrobot.MoveL(targetPos, 300)
            armrobot.MoveL(targetPos2, 200)
        
        time.sleep(5)
        sim.stopSimulation()

        # Set position of x, y, z, rx, ry, rz
        #pos = [500, -200, 400, 180, 0, 0]
        #armrobot.MoveL(pos)