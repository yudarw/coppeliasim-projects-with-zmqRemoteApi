import numpy as np
import math


def rotX(theta):
    return np.array([[1, 0, 0],
                     [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta), np.cos(theta)]])

def rotY(theta):
    return np.array([[np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])

def rotZ(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])

def getRotationMatrix(rot):
    return np.dot(np.dot(rotZ(rot[2] * math.pi / 180), rotY(rot[1] * math.pi / 180)), rotX(rot[0] * math.pi / 180))

def getEulerMatrix(rot):
    return np.dot(np.dot(rotX(rot[2] * math.pi / 180), rotY(rot[1] * math.pi / 180)), rotZ(rot[0] * math.pi / 180))

def matrix2eulerAngle(matrix):
    r11 = matrix[0, 0]
    r12 = matrix[0, 1]
    r13 = matrix[0, 2]
    r21 = matrix[1, 0]
    r22 = matrix[1, 1]
    r23 = matrix[1, 2]
    r31 = matrix[2, 0]
    r32 = matrix[2, 1]
    r33 = matrix[2, 2]
    alpha, beta, gamma = 0, 0, 0
    if r13 == 1:
        beta = 90;
        alpha = 0;
        gamma = math.atan2(r21, r31)
    else:
        beta = math.asin(r13)
        alpha = math.atan2(-r23, r33)
        gamma = math.atan2(-r12, r11)
    return [gamma * 180 / math.pi, beta * 180 / math.pi, alpha * 180 / math.pi]

def matrix2rotAngle(matrix):
    r31 = matrix[2, 0]
    r11 = matrix[0, 0]
    r12 = matrix[0, 1]
    r22 = matrix[1, 1]
    r21 = matrix[1, 0]
    r32 = matrix[2, 1]
    r33 = matrix[2, 2]

    beta = math.atan2(-r31, math.sqrt(r11 * r11 + r21 * r21))
    alpha = math.atan2(r21, r11)
    gamma = math.atan2(r32, r33)

    if beta == 90:
        alpha = 0
        gamma = math.atan2(r12, r22)
    elif beta == -90:
        alpha = 0
        gamma = -math.atan2(r12, r22)
    return [gamma * 180 / math.pi, beta * 180 / math.pi, alpha * 180 / math.pi]

def getPosFromMatrix(matrix):
    # return the 1 x 6 position and orientation vector
    pos = matrix[:3, 3]
    rot = matrix2rotAngle(matrix[:3, :3])
    euler = rot2euler(rot)
    return [pos[0], pos[1], pos[2], euler[0], euler[1], euler[2]]

def rot2euler(rot): 
    R_XYZ = getRotationMatrix(rot)
    euler = matrix2eulerAngle(R_XYZ)
    return [euler[2], euler[1], euler[0]]

def euler2rot(euler):
    R_XYZ = getEulerMatrix(euler)
    rot = matrix2rotAngle(R_XYZ)
    return [rot[2], rot[1], rot[0]]

def pose2matrix(pose):
    pos = [pose[i] for i in range(3)]
    ori = [pose[i + 3] for i in range(3)]

    rot = euler2rot(ori)
    rotM = getRotationMatrix(rot)
    matrix = np.eye(4)
    matrix[:3, :3] = rotM
    matrix[:3, 3] = pos
    return matrix

def matrixMul(matrix1, matrix2):
    # first solve the multiplication of the rotation matrix
    rot1 = matrix1[:3, :3]
    rot2 = matrix2[:3, :3]
    rot = np.dot(rot1, rot2)

    # then solve the multiplication of the position with the position
    pos1 = matrix1[:3, 3]
    pos2 = matrix2[:3, 3]
    pos = pos1 + np.dot(rot1, pos2)

    # combine the rotation matrix and the position
    matrix = np.eye(4)
    matrix[:3, :3] = rot
    matrix[:3, 3] = pos
    return matrix

def inverseMatrix(matrix):
    rot = matrix[:3, :3]
    pos = matrix[:3, 3]
    rotT = np.transpose(rot)
    pos = -np.dot(rotT, pos)
    matrix = np.eye(4)
    matrix[:3, :3] = rotT
    matrix[:3, 3] = pos
    return matrix

if __name__ == '__main__':
    pos = [500, -200, 400, 180, 0, 0]
    #matrix = getMatrix(pos)
    #print(matrix)

    #rot = [180, 0, 0]
    
    #print(getEulerMatrix(rot))