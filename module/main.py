import RFSensor
import Logics
import threading
import time
import math
import Movement
queue = []
rf = RFSensor.RFSensor(queue)
rf.start()
time.sleep(10)
before = len(queue)
while True:
    now = len(queue)
    if now > before:
        X = queue[len(queue)-1][0]
        Y = queue[len(queue)-1][1]
        angle = Logics.getAngle(X,Y)
        print("angle : "  , end = ' ')
        print(angle* (180/math.pi) )
        print("X : ", end = ' ')
        print(X, end = ' || ')
        print("Y : ", end = ' ')
        print(Y)
        print('\n')
        before = now
    #Movement.rotate((angle))
