import threading
import cv2
import numpy as np
import time
from Servo import ServoController
from ImageProcessing import image
# import onnxruntime as rt

FRAME_SIZE = 320
NAMES = ["dummy", "victim"] # Urutannya harus disesuaikan dengan yang ada di data.yaml

positioning = [4000,4000]

def ajojing():
    global positioning
    while True:
        if(positioning[0] == 4000):
            ServoController()
        else:
            KorbanTrack()
            continue

def printer():
    while True:
        if(positioning[0] != 4000):
            print(f"{positioning[0]}")
        else:
            print("None")
        time.sleep(1)

def KorbanTrack():
    while True:
        if(positioning[0] > -20 and positioning[0]<20):
            print("terdetek")
            time.sleep(1)
            time.sleep(1)
            time.sleep(1)
            time.sleep(1)
            time.sleep(1)
            break
        else:
            continue

if __name__ == '__main__':
    th1 = threading.Thread(target=printer)
    th2 = threading.Thread(target=image, args=(320, NAMES, positioning))
    th3 = threading.Thread(target=ajojing)
    th1.start()
    th2.start()
    th3.start()
    # th3.join()
    # th3.start()
    
    # for i in range(10):
    #     time.sleep(1)
    # for i in range(10):
    #     time.sleep(1)
