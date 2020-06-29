import threading
import time
import Movement
import Logics
import math
import Detector
import RFSensor

def loggingData(speed, time, count):
    distance = speed * time * count
    global gradient
    X = math.sqrt(distance**2 / (gradient**2 + 1))
    Y = X*gradient
    return X, Y

def avoid():
    global speed
    global t
    distance = 0
    target = Detector.medium_goal
    angle = Logics.getAngle(target[0],target[1]) * -2
    dist = Logics.getDistance(target[0],target[1])
    print(angle)
    Movement.rotate3(angle)
    count = 0
    for i in range(4):
        Movement.move()
    return distance, angle

queue = []
rf = RFSensor.RFSensor(queue)
rf.start()

de = threading.Thread(target = Detector.detecting)
de.start()

targetX = 0
targetY = 0
logTopX = 0
logTopY = 0
log = []
count = 0
speed = 0.2
t = 0.5
gradient = 1
while(len(queue) == 0):
    print('waiting')

targetX = queue[len(queue)-1][0]
targetY = queue[len(queue)-1][1]



if(targetY == 0): targetY = 0.1

log.append([logTopX, logTopY])
angle = Logics.getAngle(targetX, targetY)
Movement.rotate3(angle-45)

count = 0

while True:
    print(targetX, end = '   ')
    print(targetY)
    count+=1
    if(Logics.getDistance(targetX, targetY) <= 2 ):
        print('done')
        while True:
            targetX = queue[len(queue) - 1][0]
            targetY = queue[len(queue) - 1][1]
            if (Logics.getDistance(targetX, targetY) > 2 ): break

    if Detector.detect:

        Movement.stop()
        print('123123')
        tempX, tempY = loggingData(speed, t, count)
        logTopX += tempX
        logTopY += tempY

        log.append([logTopX, logTopY])

        avoid()

        targetX = queue[len(queue) - 1][0]
        targetY = queue[len(queue) - 1][1]
        angle = Logics.getAngle(targetX, targetY)
        Movement.rotate3(angle - 45)

    else:
        if(count>=5):
            before = len(queue)
            targetX = queue[len(queue) - 1][0]
            targetY = queue[len(queue) - 1][1]
            angle = Logics.getAngle(targetX, targetY)
            Movement.rotate3(angle - 45)
            count = 0
        Movement.move()
        tempX, tempY = loggingData(speed, t, 1)
        logTopX += tempX
        logTopY += tempY
        log.append([logTopX,logTopY])
