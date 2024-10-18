from coppeliasim_zmqremoteapi_client import *
import time
import math
import numpy as np

# Create a robot class
class PioneerP3DX_Robot:
    
    def __init__(self, client) -> None:
        self.sim = client.require('sim')

        # Get the handle of left and right motors
        self.motorLeft = self.sim.getObject('./leftMotor')
        self.motorRight = self.sim.getObject('./rightMotor')
        print('motorLeft handle :', self.motorLeft)
        print('motorRight handle :', self.motorRight)

        # Get ultrasonic sensor handles
        self.sensorHandles = np.zeros(16)
        for i in range(16):
            self.sensorHandles[i] = self.sim.getObject(f'./ultrasonicSensor[{i}]')
        print('sensor handles :', self.sensorHandles)

    # Move the robot forward, speed is in deg/s
    def move_forward(self, speed):
        self.sim.setJointTargetVelocity(self.motorLeft, speed * math.pi / 180)
        self.sim.setJointTargetVelocity(self.motorRight, speed * math.pi / 180)

    # move the robot backward, speed is in deg/s
    def move_backward(self, speed):
        self.sim.setJointTargetVelocity(self.motorLeft, -speed * math.pi / 180)
        self.sim.setJointTargetVelocity(self.motorRight, -speed * math.pi / 180)

    # move the robot left, speed is in deg/s
    def rotate_left(self, speed):
        self.sim.setJointTargetVelocity(self.motorLeft, -speed * math.pi / 180)
        self.sim.setJointTargetVelocity(self.motorRight, speed * math.pi / 180)
    
    # move the robot right, speed is in deg/s
    def rotate_right(self, speed):
        self.sim.setJointTargetVelocity(self.motorLeft, speed * math.pi / 180)
        self.sim.setJointTargetVelocity(self.motorRight, -speed * math.pi / 180)

    def stop(self):
        self.sim.setJointTargetVelocity(self.motorLeft, 0)
        self.sim.setJointTargetVelocity(self.motorRight, 0)

    # Control the left and right motor speed
    def move(self, vLeft, vRight):
        self.sim.setJointTargetVelocity(self.motorLeft, vLeft * math.pi / 180)
        self.sim.setJointTargetVelocity(self.motorRight, vRight * math.pi / 180)

    # Read ultrasonic sensor
    def read_ultrasonic_sensor(self):
        distances = np.zeros(16)
        for i in range(16):
            state, dist, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = self.sim.readProximitySensor(self.sensorHandles[i])
            # If not detect the object, set the sensor to its maximum value
            if dist == 0:
                dist = 1.0
            distances[i] = dist
        return distances


if __name__ == '__main__':
    client = RemoteAPIClient()
    sim = client.require('sim')
    sim.startSimulation()

    # initialize the robot
    robot = PioneerP3DX_Robot(client)

    # PID controller parameters
    kp_rot = 200
    kd_rot = 10
    ki_rot = 30
    sum_rot_error = 0
    last_rot_error = 0

    kp_trans = 90
    kd_trans = 10
    ki_trans = 0
    sum_trans_error = 0
    last_trans_error = 0

    v0 = 120                # default speed
    wall_distance = 0.25    # distance from the wall (m)
    ts = 0.05               # time_sampling (s)

    # -------------------------------------------------------------------------------------------- #
    # This is a looping that will implemet the wall following algorithm
    # We limit the time to 2 minutes    
    # -------------------------------------------------------------------------------------------- #
    start_time = time.time()
    while time.time() - start_time < 120:
        # read ultrasonic sensor
        distances = robot.read_ultrasonic_sensor()

        # map the sensor name for better reading
        front_dist = distances[4]
        #right_front_dist = (distances[7] + (distances[6] - 0.08))/2
        right_front_dist = distances[7]
        right_middle_dist = (distances[7] + distances[8]) / 2
        #right_back_dist  = (distances[8] + (distances[9] - 0.08))/2
        right_back_dist  = distances[8]
        left_front_dist  = (distances[0] + distances[1]) / 2

        # -------------------------------------------------------------------------------------------- #
        # The first PID is used to control the orientation of the robot
        # while the robot is moving forward

        rot_error = right_front_dist - right_back_dist
        sum_rot_error += rot_error
        last_rot_error = rot_error
        pid_rot = kp_rot * rot_error + ki_rot * sum_rot_error * ts + kd_rot * (rot_error - last_rot_error) / ts        

        # -------------------------------------------------------------------------------------------- #
        # The second PID is used to control the distance of the robot against the wall
        trans_error =  right_front_dist - wall_distance
        last_trans_error = trans_error
        sum_trans_error += sum_trans_error

        pid_trans = kp_trans * trans_error + ki_trans * sum_trans_error * ts + kd_trans * (trans_error - last_trans_error) / ts

        # Apply the PID to the robot
        vLeft = v0 + pid_rot + pid_trans
        vRight = v0 - pid_rot - pid_trans
        robot.move(vLeft, vRight)

        # If the front sensor is too close, rotate the robot to the left
        if(distances[4] > 0 and distances[4] < 0.4):
            robot.move(-50, 50)
            time.sleep(0.2)

        time.sleep(ts)
    
    sim.stopSimulation()