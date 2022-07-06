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

### Azure Global Variables
vSize = (2048, 1536) # This is from the Azure Kinect /rgb/image_raw topic
vFrameRate = 15.0 # This is NOT the default FPS for the Azure Kinect. Azure Kinect defaults on 5, change this in driver.launch in the drivers folder of the Azure Kinect ROS Driver
vFilename = "output/" + now.strftime("%d_%m_%Y-%H_%M_%S") + "_RGB"
vVid = cv.VideoWriter(vFilename + ".avi", fourCC, vFrameRate, vSize, True)

### Boson Global Variables
tSize = (640, 512) # This is from the Boson320 /rgb/image_raw topic
tFrameRate = 30.0
tFilename = "output/" + now.strftime("%d_%m_%Y-%H_%M_%S") + "_THERMAL"
tVid = cv.VideoWriter(tFilename + ".avi", fourCC, tFrameRate, tSize, True)

def endProgram():

    print("Shutdown initiated. Ending program.")
    vVid.release()
    tVid.release()

def visualRecorder(data):
    # Converting from Boson thermal image to OpenCV image and records it
    if(vVid.isOpened()):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data)
        img = cv.cvtColor(img, cv.COLOR_BGRA2BGR)
        vVid.write(img)
    else:
        rospy.signal_shutdown("OpenCV VideoWriter is no longer open. Program ending.")

def thermalRecorder(data):
    # Converting from Boson thermal image to OpenCV image and records it
    if(tVid.isOpened()):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data)
        img = cv.cvtColor(img, cv.COLOR_GRAY2BGR)
        tVid.write(img)
    else:
        rospy.signal_shutdown("OpenCV VideoWriter is no longer open. Program ending.")


def listener():
    rospy.init_node('recorder', anonymous=True)
    rospy.on_shutdown(endProgram)
    
    rospy.Subscriber("/rgb/image_raw", Img, visualAnalysis)
    rospy.Subscriber("/flir_boson/image_raw", Img, visualAnalysis)\

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
