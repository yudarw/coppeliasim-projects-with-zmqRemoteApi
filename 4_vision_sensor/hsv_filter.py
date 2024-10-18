from coppeliasim_zmqremoteapi_client import *
import time
import math
import numpy as np
import cv2 as cv


def nothing(x):
    pass

# Create a window
cv.namedWindow('Trackbars')

# Create trackbars for color change
cv.createTrackbar('LowerH', 'Trackbars', 0, 179, nothing)
cv.createTrackbar('UpperH', 'Trackbars', 179, 179, nothing)
cv.createTrackbar('LowerS', 'Trackbars', 0, 255, nothing)
cv.createTrackbar('UpperS', 'Trackbars', 255, 255, nothing)
cv.createTrackbar('LowerV', 'Trackbars', 0, 255, nothing)
cv.createTrackbar('UpperV', 'Trackbars', 255, 255, nothing)

def readFrame():
    # Replace this with your actual frame reading code
    buf, res = sim.getVisionSensorImg(visionSensor)
    img = np.frombuffer(buf, dtype=np.uint8).reshape(*res, 3)
    img = cv.cvtColor(img, cv.COLOR_RGB2BGR)
    img = cv.flip(img, 1)
    return img

client = RemoteAPIClient()
sim = client.require('sim')
sim.startSimulation()

visionSensor = sim.getObject('./visionSensor')
print(f'visionSensor handle : {visionSensor}')

while True:
    img = readFrame()
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # Get current positions of the trackbars
    lower_h = cv.getTrackbarPos('LowerH', 'Trackbars')
    upper_h = cv.getTrackbarPos('UpperH', 'Trackbars')
    lower_s = cv.getTrackbarPos('LowerS', 'Trackbars')
    upper_s = cv.getTrackbarPos('UpperS', 'Trackbars')
    lower_v = cv.getTrackbarPos('LowerV', 'Trackbars')
    upper_v = cv.getTrackbarPos('UpperV', 'Trackbars')

    lower_color = np.array([lower_h, lower_s, lower_v])
    upper_color = np.array([upper_h, upper_s, upper_v])

    mask = cv.inRange(hsv, lower_color, upper_color)
    result = cv.bitwise_and(img, img, mask=mask)

    cv.imshow('Original', img)
    cv.imshow('Mask', mask)
    cv.imshow('Result', result)

    if cv.waitKey(1) & 0xFF == 27:  # Press 'ESC' to exit
        break

cv.destroyAllWindows()