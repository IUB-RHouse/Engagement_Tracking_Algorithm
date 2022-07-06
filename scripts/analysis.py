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

def visualAnalysis(data):
    # Converting from Azure image to OpenCV image
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data)

    # Image Processing
    img_grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Call to appropriate files for analysis

    cv.imshow("Camera Output", img)
    cv.waitKey(1)

def listener():

    rospy.init_node('analysis', anonymous=True)
    #rospy.on_shutdown(endProgram)
    
    rospy.Subscriber("/rgb/image_raw", Img, visualAnalysis)
    rospy.Subscriber("/flir_boson/image_raw", Img, visualAnalysis)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
