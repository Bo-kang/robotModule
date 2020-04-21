import threading
import serial
import time

class RFSensor(threading.Thread,): #RF 센서의 값을 받기 위해서만 존재.
    i = 0
    tagID = ['D92E'.encode(), '5DAB'.encode(), 'C88C'.encode()] # default Tag IDs (Base on JIMBO)
    tagCount = len(tagID)
    anchorID = '8B0D'.encode()
    ACM = [None] * tagCount
    ACM_ID = [None] * tagCount

    def __init__(self):
        threading.Thread.__init__(self)

    def setTagID(self, RA, RB, RC): # if want to change RFSensors ID
        self.tagID[0] = RA.encode()
        self.tagID[1] = RB.encode()
        self.tagID[2] = RC.encode()

    def setAnchorID(self, anchorID):
        self.anchorID = anchorID

    def swap(self, ser, name):
        ser_tmp = ser[:]
        for i in range(len(name)):
            a = self.TagID.index(name[i])
            ser[a] = ser_tmp[i]
        return ser

    def initTags(self):
        print('Setting TAG System..')
        for i in range(0, self.tagCount):
            self.ACM[i] = serial.Serial("/dev/ttyACM" + str(i), 115200, timeout=5)  # check additional usb connect (ls -l /dev/ttyACM*)
        time.sleep(1)
        for i in range(self.tagCount):
            self.ACM[i].write('\r'.encode())
        time.sleep(0.5)
        for i in range(self.tagCount):
            self.ACM[i].write('\r'.encode())
        time.sleep(1)
        for i in range(self.tagCount):
            self.ACM[i].flushInput()
            self.ACM[i].flushOutput()
        time.sleep(1)
        for i in range(self.tagCount):
            self.ACM[i].write('si'.encode())  # get "system info"
            self.ACM[i].write('\n'.encode())
        time.sleep(1)
        for i in range(self.tagCount):
            for i in range(self.tagCount):
                self.ACM_ID[i] = self.ACM[i].readline()
                self.ACM_ID[i] = self.ACM_ID[i][53:57]  # pretty print "id"
        ACM = self.swap(self.ACM, self.ACM_ID)  # Set Tag in appropriate Location
        print('Complete TAG Init Serial Ports.')

        for i in range(self.tagCount):
            self.ACM[i].write('les'.encode())  # Get Distance(check document)
        time.sleep(.3)
        for i in range(self.tagCount):
            self.ACM[i].write('\n'.encode())
        time.sleep(.3)
        print('Completed TAG Setting!!')



    def run(self, RAdistList,RBdistList,RCdistList): # 원본에서 Push와 Filtering부분을 제외함.
        self.initTags()

        tagData = [None] * self.tagCount
        while True:
            # get rf distance
            for i in range(self.tagCount): #
                acmData = self.ACM[i].readline()
                acmData = acmData.split(' '.encode())
                tagData[i] = acmData[0]

            # Read and change data from each tags
            for i in range(self.tagCount):
                acmData = self.ACM[i].readline()
                acmData = acmData.split(' '.encode())
                tagData[i] = acmData[0]

            r_distance, FLAG_DISTANCE = [0.0, 0.0, 0.0], 0
            for i in range(self.tagCount):  # 1: TAG1, 2: TAG2, 3: TAG3
                if tagData[i][0:4] == self.anchorID:
                    FLAG_DISTANCE = 1
                    r_distance[i] = float(tagData[i].split('='.encode())[1])  # [Anchor name, distance]
                else:
                    print("No distance data")
                    FLAG_DISTANCE = 0

            # get rf distance
            if FLAG_DISTANCE == 1:
                RAdistList.append(r_distance[0])
                RBdistList.append(r_distance[1])
                RCdistList.append(r_distance[2])
