import threading
import time
import Movement
import Logics
import math
import Detector

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
    angle = Logics.getAngle(target[0],target[1])
    Movement.rotate3(angle)
    count = 0
    while True:
        Movement.move()
        if(count >= 5):
            distance = speed*t*count
            break
    return distance, angle



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
targetX = (int)(input("input X value : "))
targetY = (int)(input("input Y value : "))



if(targetY == 0): targetY = 0.1

log.append([logTopX, logTopY])
angle = Logics.getAngle(targetX, targetY)
Movement.rotate3(angle)



while True:
    if(logTopX + 1 >= targetX and logTopX -1 <= targetX and logTopY + 1 >= targetY and logTopY -1 <= targetY):
        break

    if Detector.detect:
        print('123123')
        tempX, tempY = loggingData(speed, t, count)
        logTopX += tempX
        logTopY += tempY

        log.append([logTopX, logTopY])

        moveDistance, angle = avoid()

        afterX = math.cos(angle) * moveDistance
        afterY = math.sin(angle) * moveDistance

        tempX = logTopX - afterX
        tempY = logTopY - afterY
        if tempY == 0: tempY = 0.1
        gradient = tempX / tempY

        angle = Logics.getAngle(tempX, tempY)
        Movement.rotate(angle)

        log.append([logTopX, logTopY])

    else:
        Movement.move()
        count += 1
        if(count >= 3):
            tempX, tempY = loggingData(speed, t, count)
            count = 0
            logTopX += tempX
            logTopY += tempY
            log.append([logTopX,logTopY])
            print(logTopX, end = ' ')
            print(logTopY)
            print(Detector.detect)
