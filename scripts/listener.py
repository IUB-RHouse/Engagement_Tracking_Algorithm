#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import sys
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as Img
from PIL import Image
from datetime import datetime

### General Global Variables
now = datetime.now()
fourCC = cv.VideoWriter_fourcc(*'MJPG')
recVis = False
recTherm = False
analyze = False

### Azure Global Variables
vSize = (2048, 1536) # This is from the Azure Kinect /rgb/image_raw topic
vFrameRate = 15.0 # This is NOT the default FPS for the Azure Kinect. Azure Kinect defaults on 5, change this in driver.launch in the drivers folder of the Azure Kinect ROS Driver
vFilename = "output/" + now.strftime("%d_%m_%Y-%H_%M_%S") + "_RGB"
vVid = cv.VideoWriter(vFilename + ".avi", fourCC, vFrameRate, vSize, True)

### Boson Global Variables
tSize = (320, 256) # This is from the Boson320 /rgb/image_raw topic
tFrameRate = 60.0
tFilename = "output/" + now.strftime("%d_%m_%Y-%H_%M_%S") + "_THERMAL"
tVid = cv.VideoWriter(tFilename + ".avi", fourCC, tFrameRate, tSize, True)

def argHandler():
    global recVis
    global recTherm
    global analyze
    global vFrameRate
    global vVid
    global fourCC
    global vSize

    for arg in sys.argv:
        if(arg[0] == '-'):
            if(arg[1:2] == "ra"):
                recVis = True
                recTherm = True
            if(arg[1:2] == "rv")
                recVis = True
            if(arg[1:2] == "rt"):
                recTherm = True
            if(arg[1] == "a"):
                analyze = True
        else:
            if(arg[0:4] == "fps="):
                vFrameRate = arg[6:7]
                vVid = cv.VideoWriter(vFilename + ".avi", fourCC, vFrameRate, vSize, True)





def endProgram():
    print("Shutdown initiated. Ending program.")
    vid.release()

def visualAnalysis(data):
    # Converting from Azure image to OpenCV image
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data)

    # Image Processing
    img_grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Call to appropriate files for analysis


    cv.imshow("Camera Output", img)
    cv.waitKey(1)

def visualRecorder(data):
    global aFrame
    # Converting from Azure image to OpenCV image and records it
    if(vid.isOpened()):
        if(not aDuration or aFrame < (int(sys.argv[2]) * int(vFrameRate))):
            bridge = CvBridge()
            img = bridge.imgmsg_to_cv2(data)
            img = cv.cvtColor(img, cv.COLOR_BGRA2BGR)
            vid.write(img)
            vFrame += 1
        else:
            print("Set aDuration reached.")
            vid.release()
    else:
        rospy.signal_shutdown("OpenCV VideoWriter is no longer open. Program ending.")

def visualRecorder(data):
    # Converting from Boson thermal image to OpenCV image and records it
    if(vid.isOpened()):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data)
        img = cv.cvtColor(img, cv.COLOR_BGRA2BGR)
        vVid.write(img)
    else:
        rospy.signal_shutdown("OpenCV VideoWriter is no longer open. Program ending.")

def thermalRecorder(data):
    # Converting from Boson thermal image to OpenCV image and records it
    if(vid.isOpened()):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data)
        img = cv.cvtColor(img, cv.COLOR_BGRA2BGR)
        tVid.write(img)
    else:
        rospy.signal_shutdown("OpenCV VideoWriter is no longer open. Program ending.")


def listener():
    global recVis
    global recTherm
    global analyze

    rospy.init_node('listener', anonymous=True)
    rospy.on_shutdown(endProgram)

    if(recVis):
        rospy.Subscriber("/rgb/image_raw", Img, visualRecorder)
    if(recTherm):
        rospy.Subscriber("/flir_boson/image_raw", Img, thermalRecorder)
    if(analyze):
        rospy.Subscriber("/rgb/image_raw", Img, visualAnalysis)
        rospy.Subscriber("/flir_boson/image_raw", Img, visualAnalysis)
    else:
        print("Error: Argument(s) not recognized.")
        sys.exit()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    if(len(sys.argv) < 2):
        print("Error: Mode argument required")
        sys.exit()
    else:
        argHandler()
        listener()
