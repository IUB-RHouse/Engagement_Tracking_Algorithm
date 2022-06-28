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

frame = 0
size = (2048, 1536) # This is from the Azure Kinect /rgb/image_raw topic
frameRate = 15.0 # This is NOT the default FPS for the Azure Kinect. Azure Kinect defaults on 5, change this in driver.launch in the drivers folder of the Azure Kinect ROS Driver
now = datetime.now()
fileName = "output/" + now.strftime("%d_%m_%Y-%H_%M_%S")
fourcc = cv.VideoWriter_fourcc(*'MJPG')
vid = cv.VideoWriter(fileName + ".avi", fourcc, frameRate, size, True)

def endProgram():
    print("Shutdown initiated. Ending program. Please press \'ctrl+C\'")
    vid.release()

def visualAnalysis(data):
    # Converting from Azure image to OpenCV image
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data)

    # Image Processing
    img_grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Call to appropriate files for analysis
    # Shekinah will implement facial detection here

    cv.imshow("Camera Output", img)
    cv.waitKey(1)

def visualRecorder(data):
    global frame
    # Converting from Azure image to OpenCV image
    if(vid.isOpened() and frame < (int(sys.argv[2]) * int(frameRate))):
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data)
        img = cv.cvtColor(img, cv.COLOR_BGRA2BGR)
        vid.write(img)
        frame += 1
    else:
        print("Set duration reached. Ending program. Please press \'ctrl+C\'")
        vid.release()


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.on_shutdown(endProgram)

    if(sys.argv[1] == "-r"):
        rospy.Subscriber("/rgb/image_raw", Img, visualRecorder)
    elif(sys.argv[1] == "-a"):
        rospy.Subscriber("/rgb/image_raw", Img, visualAnalysis)
    else:
        print("Error: Argument \'" + sys.argv[1] + "\' not recognized.")
        sys.exit()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    if(len(sys.argv) > 1):
        print("Error - Mode argument required")
        sys.exit()
    else:
        listener()
