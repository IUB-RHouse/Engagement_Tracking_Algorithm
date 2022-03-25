#!/usr/bin/env python3.6
import rospy
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as Img
from PIL import Image


def callback(data):
    # Converting from Azure image to OpenCV image
    print("HIT3")
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(data)

    # Image Processing
    img_grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Loading Haar Cascade classifier for frontal face
    haar_frontal_face = cv.CascadeClassifier(cv.data.haarcascades + 'include/haarcascade_frontalface_default.xml')
    # Setting face square parameters
    face_rects = haar_frontal_face.detectMultiScale(img_grey, scaleFactor=1.2, minNeighbors=5, minSize=(30, 30), flags=cv.CASCADE_SCALE_IMAGE)
    # Print number of faces found
    print("Detected faces: ", len(face_rects))

    # Draw rectangules over images
    for (nx, ny, x, y) in face_rects:
        cv.rectangle(img, (nx, ny), (nx + x, ny + y), (0, 0, 255), 2)
    print("HIT4")
    cv.imshow("Camera Output", img)
    cv.waitKey(1)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    print("Hit1")
    rospy.init_node('listener', anonymous=True)
    print("Hit2")
    rospy.Subscriber("/rgb/image_raw", Img, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
