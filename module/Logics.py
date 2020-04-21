import numpy as np
import math

distance = 43
frontAngle = math.atan(1)

def filter(array):
    arrSize = len(array)
    array.sort()
    return np.mean(array[1:-2])

def calcCoordinate(avgTag1, avgTag2, avgTag3):
    X_COORD_M = ((avgTag1 ** 2) - (avgTag2 ** 2) + (distance ** 2)) / (2 * distance) - distance / 2
    Y_COORD_M = ((avgTag1 ** 2) - (avgTag3 ** 2) + (distance ** 2)) / (2 * distance) - distance / 2
    return X_COORD_M, Y_COORD_M

def getAngle(X, Y):
    dist = math.sqrt(X**2 + Y**2)
    curAngle = math.asin(Y/dist)
    resultAngle = 0
    if(X < 0 and Y < 0):
        curAngle = curAngle * -1
        resultAngle = math.pi
    elif(X < 0):
        resultAngle = math.py*0.5
    rotateAngle = frontAngle - curAngle ## 회전 방향을 고려하여 부호를 변경해야할 필요가 있음!!
    resultAngle += rotateAngle

    return resultAngle