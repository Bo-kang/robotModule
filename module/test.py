import Movement
import Logics
import math
import threading
import time
import test1


t = threading.Thread(target=test1.th)
t.start()
while True:
    if(test1.check):
        print('True')
    else :
        print('false')
    time.sleep(0.5)