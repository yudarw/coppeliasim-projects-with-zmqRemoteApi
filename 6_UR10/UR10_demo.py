from coppeliasim_zmqremoteapi_client import *
import math
import time
from interpolation import linear_interpolation, joint_interpolation
from homogeneous_transform import *
import numpy as np

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

        self.simJoints = [self.sim.getObject(f'/{robotName}/joint{i + 1}') for i in range(6)]

        # Initialize the UR10 inverse kinematic
        self.ikEnv = self.simIK.createEnvironment()
        self.ikGroup = self.simIK.createGroup(self.ikEnv)
        self.simIK.addElementFromScene(self.ikEnv, self.ikGroup, self.simrobot, self.simTip, self.simTarget, self.simIK.constraint_pose)

    # Get the position of the object relative to the robot base
    def GetObjectPosition(self, objectName):
        handle = sim.getObject(f'/{objectName}')
        pos = self.sim.getObjectPosition(handle, self.simrobot)
        ori = self.sim.getObjectOrientation(handle, self.simrobot)
        pos = [pos[i] * 1000 for i in range(3)]
        ori = [ori[i] * 180 / math.pi for i in range(3)]
        return [pos[0],pos[1],pos[2],ori[0],ori[1],ori[2]]

    def GetObjectMatrix(self, objectName):
        handle = sim.getObject(f'/{objectName}')
        # matrix: array of 12 values [Vx0 Vy0 Vz0 P0 Vx1 Vy1 Vz1 P1 Vx2 Vy2 Vz2 P2]
        _matrix = sim.getObjectMatrix(handle, self.simrobot)
        simMatrix = np.eye(4)
        # convert the matrix to 4x4 matrix
        for i in range(3):
            simMatrix[i, :] = _matrix[i*4:i*4+4]
        return simMatrix

    # Read the cartesion position of the robot
    def ReadPosition(self):
        pos = self.sim.getObjectPosition(self.simTip, self.simrobot)
        ori = self.sim.getObjectOrientation(self.simTip, self.simrobot)
        pos = [pos[i] * 1000 for i in range(3)]
        ori = [ori[i] * 180 / math.pi for i in range(3)]
        return [pos[0],pos[1],pos[2],ori[0],ori[1],ori[2]]
    
    def ReadJointPosition(self):
        jointPos = [self.sim.getJointPosition(joint) * 180 / math.pi for joint in self.simJoints]
        return jointPos

    # Set robot position in mm and deg
    def SetPosition(self, targetPos):
        pos = [targetPos[i] / 1000 for i in range(3)]
        ori = [targetPos[i + 3] * math.pi / 180 for i in range(3)]
        self.sim.setObjectPosition(self.simTarget, pos, self.simrobot)
        self.sim.setObjectOrientation(self.simTarget, ori, self.simrobot)
        # Apply ik
        #self.simIK.handleGroup(self.ikEnv, self.ikGroup, {'syncWorlds' : True, 'allowError' : True})
        self.simIK.applyIkEnvironmentToScene(self.ikEnv, self.ikGroup)
    
    def SetJointPosition(self, targetJointPos):
        targetJointPos = [joint * math.pi / 180 for joint in targetJointPos]
        for i in range(len(self.simJoints)):
            self.sim.setJointPosition(self.simJoints[i], targetJointPos[i])
        # Apply ik
        #self.simIK.handleGroup(self.ikEnv, self.ikGroup, {'syncWorlds' : True, 'allowError' : True})
        #self.simIK.applyIkEnvironmentToScene(self.ikEnv, self.ikGroup)

    def MoveL(self, targetPos, speed):
        initialPos = self.ReadPosition()
        interpolated_points, steps, time_per_step = linear_interpolation(initialPos, targetPos, speed, speed/15)
        for point in interpolated_points:
            self.SetPosition(point)        
            if (time_per_step < 0.05):
                time_per_step = 0.05
            time.sleep(time_per_step)
    

    def MoveJ(self, targetJointPos, speed):
        initialJointPos = self.ReadJointPosition()
        interpolated_points, steps, time_per_step = joint_interpolation(initialJointPos, targetJointPos, speed)
        for point in interpolated_points:
            self.SetJointPosition(point)
            time.sleep(time_per_step)

    # Linear movement by using coppeliasim moveToPose
    def MoveL_2(self, targetPos, speed):
        pass

    
    def MoveToConfig(self):
        pass

if __name__ == '__main__':
        
        armrobot = UR10(client, 'UR3')

        intialpos = armrobot.ReadPosition()
        targetPos = armrobot.GetObjectPosition('target1')
        targetPos2 = armrobot.GetObjectPosition('target2')
        

        # Move the robot in the joint space
        #targetJoint = [0, 15, 90, 0, 0, 0]
        #armrobot.MoveJ(targetJoint, 30 * math.pi / 180)


        # ============================================================================== #
        # Now the problem that we wanna solve is to define the workframe of the robot
        user_frame = armrobot.GetObjectPosition('Cuboid/origin')
        T_AB = pose2matrix(user_frame)


        for i in range(10):
            pos_from_user_frame = [0, 0, 0, 180, 0, 180]
            T_BC = pose2matrix(pos_from_user_frame)
            T_AC = np.dot(T_AB, T_BC)
            endpos = getPosFromMatrix(T_AC)
            armrobot.MoveL(endpos, 300)

            pos_from_user_frame = [200, 0, 0, 180, 0, 90]
            T_BC = pose2matrix(pos_from_user_frame)
            T_AC = np.dot(T_AB, T_BC)
            endpos = getPosFromMatrix(T_AC)
            armrobot.MoveL(endpos, 300)

            pos_from_user_frame = [200, 200, 0, 180, 0, 0]
            T_BC = pose2matrix(pos_from_user_frame)
            T_AC = np.dot(T_AB, T_BC)
            endpos = getPosFromMatrix(T_AC)
            armrobot.MoveL(endpos, 300)

            pos_from_user_frame = [0, 200, 0, 180, 0, 0]
            T_BC = pose2matrix(pos_from_user_frame)
            T_AC = np.dot(T_AB, T_BC)
            endpos = getPosFromMatrix(T_AC)
            armrobot.MoveL(endpos, 300)
        # ============================================================================== #
        # simMatrix = armrobot.GetObjectMatrix('target1')
        # print(f'target position: {targetPos}')

        # print('sim calculation:')
        # print(simMatrix)

        # print('manual calculation:')
        # simMatrix = pose2matrix(targetPos)
        # print(simMatrix)

        # rotAngle = matrix2rotAngle(simMatrix)
        # eulerAngle = rot2euler(rotAngle)
        # print(eulerAngle)

        #while True:
        #    armrobot.MoveL(targetPos, 300)
        #    armrobot.MoveL(targetPos2, 200)
        #    armrobot.MoveL(targetPos, 300)
        #    armrobot.MoveL(targetPos2, 200)
        
        time.sleep(5)
        sim.stopSimulation()

        # Set position of x, y, z, rx, ry, rz
        #pos = [500, -200, 400, 180, 0, 0]
        #armrobot.MoveL(pos)