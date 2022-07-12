# -*- coding: utf-8 -*-
'''
The program will do the following steps:
1. find location of nose on rgb image
2. color the nose part as white
3. save the image
'''
import os
import cv2
import numpy as np
from proctoring.face_detector import get_face_detector, find_faces
from proctoring.face_landmarks import get_landmark_model, detect_marks


def feature_on_mask(mask, side, shape):
    """
    Create ROI on mask of the size of eyes and also find the extreme points of select feature

    Parameters
    ----------
    mask : np.uint8
        Blank mask to draw eyes on
    side : list of int
        the facial landmark numbers of eyes
    shape : Array of uint32
        Facial landmarks

    Returns
    -------
    mask : np.uint8
        Mask with region of interest drawn
    [l, t, r, b] : list
        left, top, right, and bottommost points of ROI

    """
    points = [shape[i] for i in side]
    points = np.array(points, dtype=np.int32)
    mask = cv2.fillConvexPoly(mask, points, 255)
    l = points[0][0]
    t = (points[1][1] + points[2][1]) // 2
    r = points[3][0]
    b = (points[4][1] + points[5][1]) // 2
    return mask, [l, t, r, b]


############################################
def draw_rectangle(img2, rect, save_dir, img_name):
    img2[rect[1]: rect[3] + 1, rect[0]] = [255, 255, 255]
    img2[rect[1]: rect[3] + 1:, rect[2]] = [255, 255, 255]
    img2[rect[1], rect[0]: rect[2] + 1] = [255, 255, 255]
    img2[rect[3], rect[0]: rect[2] + 1] = [255, 255, 255]
    return img2


def img_nose_label(img, img_name, face_model, nose_point, save=True, save_dir='pic_test/test/', save_frame='test_nose_{}.jpg'):
    rects = find_faces(img, face_model)
    if not rects:
        if save:
            return
        else:
            return None, None
    rect = rects[0]
    for edg in range(4):
        rect[edg] = max(0, rect[edg])
    img = draw_rectangle(img2=img, rect=rect, save_dir=save_dir, img_name=img_name)

    shape = detect_marks(img, landmark_model, rect)
    mask = np.zeros(img.shape[:2], dtype=np.uint8)
    mask, nose_points_find = feature_on_mask(mask, nose_point, shape)
    kernel = np.ones((9, 9), np.uint8)
    mask = cv2.dilate(mask, kernel, 5)

    nose_find = cv2.bitwise_and(img, img, mask=mask)
    if save:
        for i in range(len(nose_find)):
            for j in range(len(nose_find[i])):
                if sum(nose_find[i][j]) != 0:
                    img[i][j] = [255, 255, 255]

        cv2.imwrite('{}{}'.format(save_dir, save_frame.format(img_name)), img)
        return
    else:
        return img, nose_find


#######################################################################################
if __name__ == '__main__':
    os.chdir('RHouse/Proctoring')
    face_model = get_face_detector()
    landmark_model = get_landmark_model()
    nose_label = [28, 29, 30, 31, 32, 33, 34, 35]

    # test img
    pic_dir = 'pic_test/'
    for file in os.listdir(pic_dir):
        if file.endswith('.jpg') or file.endswith('.png'):
            img_name = file
            img = cv2.imread('{}{}'.format(pic_dir, img_name))
            img_nose_label(img, img_name, face_model, nose_point=nose_label, save=True)

