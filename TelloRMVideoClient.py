import threading
import signal
import opus_decoder
import pyaudio
import os
import sys
from PIL import Image as PImage
import cv2
import numpy as np
import queue
import time
import signal
from rm import RMS1
from tello import Tello

class TelloRMVideoClient:

    def __init__(self,rm,tello):
        self.rm = rm
        self.tello = tello
        self.frame = None
        self.videothread = threading.Thread(target=self.videoLoop, args=())
        self.rmaudiothread = threading.Thread(target=self.audioLoop, args=())
        self.stopEvent = True
        self.rmReady = False
        self.telloReady = False        
        self.runTellothread = threading.Thread(target=self.telloLoop, args=())
        self.runRMthread = threading.Thread(target=self.rmLoop, args=())
        self.stopApp = False

    def telloLoop(self):
# function to control action of tello. put your robotmaster code here in this telloloop function

        self.tello.send('streamon')
        self.telloReady = True
# put your tello code starting from here
        while self.stopEvent:
            time.sleep(0.5)
        #codes to instruct tello flying start here#
        self.tello.send('takeoff')
        self.tello.send('forward 50')
        self.tello.send('mon')
        self.tello.send('left 100')
        i = 0
        while i < 20:
            time.sleep(2)
            print('tof: %s' % str(self.tello.tof))
            print('bat: %s' % str(self.tello.batterylevel))
            i = i + 1
        self.tello.send('land')
        i = 0
        while i < 5:
            time.sleep(1)
            i = i + 1
            #codes to instruct tello flying ends here#
# put your tello code ends here
        self.telloReady = False

    def rmLoop(self):
# loop to control action of robomaster. put your robotmaster code here in this rmloop function

        self.rm.send('stream on')    
        while self.rm.in_video_mode == False:
            time.sleep(0.5)
#        self.rm.send('audio on')    
#        while self.rm.in_audio_mode == False:
#            time.sleep(0.5)

        self.rmReady = True

        while self.stopEvent:
            time.sleep(0.5)
# put your robomaster code starting from here

        self.rm.send('chassis move x 0.1 y 0.2')

        self.rm.send('chassis move z 90')
        self.rm.send('chassis move x 0.1 y 0.2')
        i = 0
        while i < 5:
            time.sleep(1)
            i = i + 1
        self.rm.send('chassis move z 180')
        self.rm.send('chassis move z 180')
        i = 0
        while i < 10:
            time.sleep(1)
            i = i + 1
        self.rm.send('chassis move x 0.1 y 0.2')
        while i < 10:
            time.sleep(1)
            i = i + 1
# put your robomaster code ends here
        self.rmReady = False

    def start(self):
        self.runTellothread.start()
        self.runRMthread.start()

        while self.telloReady == False or self.rmReady == False:
            time.sleep(0.5)
        self.videothread.start()
#        self.rmaudiothread.start()

        self.stopEvent = False

        while self.telloReady or self.rmReady:
            time.sleep(0.5)
        
        self.stopApp = True

    def videoLoop(self):
        while self.stopApp == False:
            if self.rmReady:
                self.rframe = self.rm.readframe()
            
            if self.telloReady:
                self.tframe = self.tello.readframe()

            if self.rframe is None or self.rframe.size == 0 or self.tframe is None or self.tframe.size == 0:
                continue

            cv2.namedWindow("RMLiveview")
            rimage = PImage.fromarray(self.rframe)
            timage = PImage.fromarray(self.tframe)
            rimg = cv2.cvtColor(np.array(rimage), cv2.COLOR_RGB2BGR)
            timg = cv2.cvtColor(np.array(timage), cv2.COLOR_RGB2BGR)

            cv2.imshow("RMLiveview", rimg)
            cv2.imshow("TelloLiveview", timg)
            cv2.waitKey(1)
            #computer vision or AI codes can be here
            
            #computer vision or AI codes end here

    def audioLoop(self):
        p = pyaudio.PyAudio()

        stream = p.open(format=pyaudio.paInt16,channels=1,rate=48000,output=True)

        while self.stopApp == False:
            a_output = self.rm.readaudioframe()

            if a_output:
                stream.write(a_output)
            else:
                print("audio stream empty")

        stream.stop_stream()
        stream.close()

    def onClose(self):
        print("Close window")
        self.stopEvent = True
        self.rmaudiothread.join()
        self.videothread.join()
        self.rm.video_freeze()
        self.tello.video_freeze()
        self.rm.close()
        self.tello.close()
    



