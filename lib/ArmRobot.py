from coppeliasim_zmqremoteapi_client import *
import math
import time
from interpolation import linear_interpolation, joint_interpolation
from homogeneous_transform import *
import numpy as np

# Universal Robot Class
class UniversalRobot:
    def __init__(self, robot_name):
        client = RemoteAPIClient()
        self.sim = client.require('sim')
        self.simIK = client.require('simIK')
        self.robotName = robot_name

        # Initialize robot arm
        self.simRobot = self.sim.getObject(f'/{robot_name}')
        self.simTip = self.sim.getObject(f'/{robot_name}/ikTip')
        self.simTarget = self.sim.getObject(f'/{robot_name}/ikTarget')

        # Get robot joints handle
        self.simJoints = []
        for i in range(6):
            object_name = f"/{robot_name}/joint{i + 1}"
            self.simJoints.append(self.sim.getObject(object_name))

        # Initialize the UR10 inverse kinematic
        self.ikEnv = self.simIK.createEnvironment()
        self.ikGroup = self.simIK.createGroup(self.ikEnv)
        self.simIK.addElementFromScene(self.ikEnv, self.ikGroup, self.simRobot, self.simTip, self.simTarget, self.simIK.constraint_pose)

        # Inverse Kinematic
        self.ikMaxVel = 0.2
        self.ikMaxAccel = 0.1
        self.ikMaxJerk = 0.1


    # Get the position of the object relative to the robot base
    def GetObjectPosition(self, objectName):
        handle = self.sim.getObject(f'/{objectName}')
        pos = self.sim.getObjectPosition(handle, self.simRobot)
        ori = self.sim.getObjectOrientation(handle, self.simRobot)
        pos = [pos[i] * 1000 for i in range(3)]
        ori = [ori[i] * 180 / math.pi for i in range(3)]
        return [pos[0],pos[1],pos[2],ori[0],ori[1],ori[2]]
    
    # Get object 4x4 object matrix
    def GetObjectMatrix(self, objectName):
        handle = self.sim.getObject(f'/{objectName}')
        # matrix: array of 12 values [Vx0 Vy0 Vz0 P0 Vx1 Vy1 Vz1 P1 Vx2 Vy2 Vz2 P2]
        _matrix = self.sim.getObjectMatrix(handle, self.simRobot)
        simMatrix = np.eye(4)
        # convert the matrix to 4x4 matrix
        for i in range(3):
            simMatrix[i, :] = _matrix[i*4:i*4+4]
        return simMatrix
    
    # Read the cartesion position of the robot
    def ReadPosition(self):
        pos = self.sim.getObjectPosition(self.simTip, self.simRobot)
        ori = self.sim.getObjectOrientation(self.simTip, self.simRobot)
        pos = [pos[i] * 1000 for i in range(3)]
        ori = [ori[i] * 180 / math.pi for i in range(3)]
        return [pos[0],pos[1],pos[2],ori[0],ori[1],ori[2]]
    
    # Read the joint position of the robot (deg)    
    def ReadJointPosition(self):
        jointPos = [self.sim.getJointPosition(joint) * 180 / math.pi for joint in self.simJoints]
        return jointPos
    
    # Direct set robot end-effector position (mm) and orientation (deg)
    def SetPosition(self, targetPos):
        pos = [targetPos[i] / 1000 for i in range(3)]
        ori = [targetPos[i + 3] * math.pi / 180 for i in range(3)]
        self.sim.setObjectPosition(self.simTarget, pos, self.simRobot)
        self.sim.setObjectOrientation(self.simTarget, ori, self.simRobot)
        # Apply ik
        #self.simIK.handleGroup(self.ikEnv, self.ikGroup, {'syncWorlds' : True, 'allowError' : True})
        self.simIK.applyIkEnvironmentToScene(self.ikEnv, self.ikGroup)

    # Set joint position
    def SetJointPosition(self, targetJointPos):
        targetJointPos = [joint * math.pi / 180 for joint in targetJointPos]
        for i in range(len(self.simJoints)):
            self.sim.setJointPosition(self.simJoints[i], targetJointPos[i])

    # Move linear using interpolation
    def LegacyMoveL(self, targetPos, speed):
        initialPos = self.ReadPosition()
        interpolated_points, steps, time_per_step = linear_interpolation(initialPos, targetPos, speed, speed/15)
        for point in interpolated_points:
            self.SetPosition(point)        
            if (time_per_step < 0.05):
                time_per_step = 0.05
            time.sleep(time_per_step)
    
    def SetSpeed(self, speed):
        self.ikMaxVel = speed / 1000
        self.ikMaxAccel = self.ikMaxVel * 2
        self.ikMaxJerk = self.ikMaxVel * 2
   
    def ikCallback(self,target_quaternion):
        self.sim.setObjectPose(self.simTarget, -1, target_quaternion)
        self.simIK.applyIkEnvironmentToScene(self.ikEnv, self.ikGroup)

    # Move linear using Coppeliasim MoveL function
    def MoveL(self, targetPos, speed):
        self.SetSpeed(speed)

        pos = [targetPos[0] / 1000, targetPos[1] / 1000, targetPos[2] / 1000]
        ori = [targetPos[3] * 3.14 / 180, targetPos[4] * 3.14 / 180, targetPos[5] * 3.14 / 180]
        self.sim.setObjectPosition(self.simTarget, self.simRobot, pos)
        self.sim.setObjectOrientation(self.simTarget, self.simRobot, ori)
        target_quaternion = self.sim.getObjectPose(self.simTarget, -1)
        current_quaternion = self.sim.getObjectPose(self.simTip, -1)
        self.sim.moveToPose(-1, current_quaternion, self.ikMaxVel, self.ikMaxAccel, self.ikMaxJerk, target_quaternion, self.ikCallback, None, None)




if __name__ == '__main__':
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.startSimulation()

    # Initialize robot instance
    robot = UniversalRobot('UR3')
    
    # Get object position
    initPos = robot.ReadPosition()
    print(f"robot initial position: {initPos}")
    
    # Get target position
    targetPosition1 = robot.GetObjectPosition('target1')
    print(f"position of target 1: {targetPosition1}")
    
    targetPosition2 = robot.GetObjectPosition('target2')
    print(f"position of target 2: {targetPosition2}")

    robot.MoveL(targetPosition1, 100)

    