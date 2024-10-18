from coppeliasim_zmqremoteapi_client import *
import time
import math
import numpy as np
import cv2 as cv

# ----------------- Read vision sensor ----------------- #
def readFrame():
    #sim.handleVisionSensor(visionSensor)
    buf, res = sim.getVisionSensorImg(visionSensor)
    img = np.frombuffer(buf, dtype=np.uint8).reshape(*res, 3)
    img = cv.cvtColor(img, cv.COLOR_RGB2BGR)
    # flip the image horizontally
    img = cv.flip(img, 1)
    return img

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
    
    # Detect the object:
    contours, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
    cx, cy = 0, 0
    for contour in contours:
        if cv.contourArea(contour) > 500:  # Filter small contours
            x, y, w, h = cv.boundingRect(contour)  
            cx, cy = x + w // 2, y + h // 2  # Calculate center coordinates
            cv.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 1)
            cv.putText(img, f'{color} ({cx}, {cy})', (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            break
    return img, len(contours), cx, cy



# ----------------- Main ----------------- #
client = RemoteAPIClient()
sim = client.require('sim')
sim.startSimulation()

# Get vision sensor handle
visionSensor = sim.getObject('./visionSensor')
print(f'visionSensor handle : {visionSensor}')

color = ['red', 'green', 'blue', 'yellow']

while True:
    img = readFrame()
    for i in range(4):
        img, n, x, y = filter_color(img, color[i])
        if n > 0:
            break    
    cv.imshow('img', img)
    cv.waitKey(1)
    