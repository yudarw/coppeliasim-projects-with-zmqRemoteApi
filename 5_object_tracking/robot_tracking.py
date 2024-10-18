from coppeliasim_zmqremoteapi_client import *
import time
import math
import numpy as np
import cv2 as cv

# ----------------- Create a robot class ----------------- #
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
                dist = 2.0
            distances[i] = dist
        return distances


# ----------------- Read vision sensor ----------------- #
def readFrame():
    #sim.handleVisionSensor(visionSensor)
    buf, res = sim.getVisionSensorImg(visionSensor)
    img = np.frombuffer(buf, dtype=np.uint8).reshape(*res, 3)
    img = cv.cvtColor(img, cv.COLOR_RGB2BGR)
    # flip the image horizontally
    img = cv.flip(img, 1)
    return img

def showImage(k, img):
    cv.imshow(f'img{k}', img)
    cv.waitKey(1)

# Perform color filtering
def filter_color(img, color):

    if color == 'red':
        lower_color = np.array([0, 70, 50])
        upper_color = np.array([10, 255, 255])
    elif color == 'green':
        lower_color = np.array([36, 25, 25])
        upper_color = np.array([70, 255, 255])
    elif color == 'blue':
        lower_color = np.array([105, 70, 50])
        upper_color = np.array([130, 255, 255])
    elif color == 'yellow':
        lower_color = np.array([25, 50, 70])
        upper_color = np.array([35, 255, 255])

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lower_color, upper_color)
    cv.imshow('Mask', mask)
    # Detect the object:
    contours, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
    cx, cy = 0, 0
    objsize = 0
    objDetected = False
    for contour in contours:
        if cv.contourArea(contour) > 100:  # Filter small contours
            objDetected = True
            objsize = cv.contourArea(contour)
            x, y, w, h = cv.boundingRect(contour)  
            cx, cy = x + w // 2, y + h // 2  # Calculate center coordinates
            cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 1)
            cv.putText(img, f'{color} ({cx}, {cy})', (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            break
    return img, objDetected, cx, cy, objsize



# ----------------- Main ----------------- #
client = RemoteAPIClient()
sim = client.require('sim')
sim.startSimulation()

# Initialize the robot
robot = PioneerP3DX_Robot(client)

# Get vision sensor handle
visionSensor = sim.getObject('./visionSensor')
print(f'visionSensor handle : {visionSensor}')

color = ['red', 'green', 'blue', 'yellow']

# Rotate the robot to detect the object
robot.rotate_left(20)
i = 0
while True:
    # Get the image from the vision sensor
    img = readFrame()
    img, detectObj, x, y, objsize = filter_color(img, color[i])
    cv.imshow('img', img)

    # if detect the object, move the robot toward the object
    if detectObj:
        error = 128 - x                 # calculte the error from the center of the image
        p = 0.4 * error                 # simply apply a proportional controller
        robot.move(150 - p, 150 + p)    # Move the robot forward with speed 150

        # If close to the object, rotate the robot to the left then continue to scan the next color
        if objsize > 15000:
            i = (i + 1) % 4
            robot.rotate_left(50)
            time.sleep(1)
    else:
        robot.rotate_left(50)
                
    # If keyboard e is press exit the loop:
    if cv.waitKey(1) & 0xFF == ord('e'):
        break
    