from coppeliasim_zmqremoteapi_client import *

client = RemoteAPIClient()
sim = client.require('sim')
        

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
        # ini 
        pass

if __name__ == '__main__':
        armrobot = UR10(client, 'UR10')