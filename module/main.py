import RFSensor
import Logics
import threading
import time
import math
import Movement
queue = []
rf = RFSensor.RFSensor(queue)
rf.start()
detect = False
before = len(queue)
targetX = 0
targetY = 0

while True:
    now = len(queue)
    X = 0
    Y = 0
    if (now != before):
        before = now
        targetX = queue[now-1][0]
        targetY = queue[now-1][1]
        angle = Logics.getAngle(targetX, targetY)
        print('angle', end = ' : ')
        print(angle)

    if targetX != 0 and targetY != 0:
        angle = Logics.getAngle(targetX, targetY)

        print('angle', end = ' : ')
        print(angle)

        distance = Logics.getDistance(targetX,targetY)
        Movement.rotate3(angle)
        distance -= 150
        time = distance/20
        targetCount = time/0.5

        print('targetCount', end=' : ')
        print(targetCount)

        print('distance', end = ' : ')
        print(distance)

        count = 0
        while not detect:
            Movement.move()
            if(targetCount <= count):
                break
            count += 1
            print('count', end=' : ')
            print(count)

        if not detect : break







