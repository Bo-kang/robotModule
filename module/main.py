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
        print(targetX)
        print(targetY)








