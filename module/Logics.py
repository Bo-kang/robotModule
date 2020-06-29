import numpy as np
import math
import sys
distance = 0.32
frontAngle = math.atan(1)

def filter(array):
    array.sort()
    return np.mean(array[1:-2])

def calcCoordinate(avgTag1, avgTag2, avgTag3):
    X_COORD_M = ((avgTag1 ** 2) - (avgTag2 ** 2) + (distance ** 2)) / (2 * distance) - distance / 2
    Y_COORD_M = ((avgTag1 ** 2) - (avgTag3 ** 2) + (distance ** 2)) / (2 * distance) - distance / 2
    return X_COORD_M, Y_COORD_M

def getDistance(X,Y):
    return math.sqrt(X**2 + Y**2)

def getAngle(x, y):
    if y > 0:
        return math.atan(x / y) * 180 / math.pi  # degree
    else:
        if x > 0:
            return 180 + math.atan(x / (y + sys.float_info.epsilon)) * 180 / math.pi  # degree
        else:
            return - 180 + math.atan(x / (y + sys.float_info.epsilon)) * 180 / math.pi  # degree