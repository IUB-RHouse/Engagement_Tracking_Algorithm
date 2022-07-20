# -*- coding: utf-8 -*-
'''
The program will do the following steps:
1. find face rectangle on rgb image
2. find location of nose on rgb image
3. color the nose part as white
4. save the image
'''
import os
import cv2
import numpy as np
from proctoring.face_detector import get_face_detector, find_faces
from proctoring.face_landmarks import get_landmark_model, detect_marks


def try_make_dir(dir1):
    try:
        os.mkdir(dir1)
        print('create new dir: {}'.format(dir1))
    except:
        pass


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
    if len(points) == 1:
        return mask, [points[0][0], points[0][1], points[0][0], points[0][1]]
    else:
        l = points[0][0]
        t = (points[1][1] + points[2][1]) // 2
        r = points[3][0]
        b = (points[4][1] + points[5][1]) // 2
    return mask, [l, t, r, b]


############################################
def draw_rectangle(img2, rect):
    img2[rect[1]: rect[3] + 1, rect[0]] = [255, 255, 255]
    img2[rect[1]: rect[3] + 1:, rect[2]] = [255, 255, 255]
    img2[rect[1], rect[0]: rect[2] + 1] = [255, 255, 255]
    img2[rect[3], rect[0]: rect[2] + 1] = [255, 255, 255]
    return img2


def img_nose_label(img, img_name, face_model, nose_point, save=True, save_dir='pic_test/test/', save_frame='test_nose_{}', img_type='jpg'):
    rects = find_faces(img, face_model)
    if not rects:
        if save:
            return
        else:
            return None, None
    rect = rects[0]
    for edg in range(4):
        rect[edg] = max(0, rect[edg])
    img = draw_rectangle(img2=img, rect=rect)

    shape = detect_marks(img, landmark_model, rect)
    mask = np.zeros(img.shape[:2], dtype=np.uint8)
    mask, nose_points_find = feature_on_mask(mask, nose_point, shape)
    kernel = np.ones((9, 9), np.uint8)
    mask = cv2.dilate(mask, kernel, 5)

    nose_find = cv2.bitwise_and(img, img, mask=mask)
    if save:
        for dir_i in [save_dir, save_dir + 'rectangle_info/']:
            try_make_dir(dir_i)
        # Save img
        non_zero = np.nonzero(nose_find)
        '''
        non_zero = (array([165, 165, 165, ..., 188, 188, 188], dtype=int64), array([717, 717, 717, ..., 728, 728, 728], dtype=int64), array([0, 1, 2, ..., 0, 1, 2], dtype=int64))
        '''
        i = 0
        while i < len(non_zero[0]):
            img[non_zero[0][i]][non_zero[1][i]] = [255, 255, 255]
            i += 3
        cv2.imwrite('{}{}'.format(save_dir, save_frame.format(img_name)), img)
        #  Save rectangle info
        rect_info = {'rect': rect, 'nose_area': non_zero, 'nosetip_point': nose_points_find}
        np.save(save_dir + 'rectangle_info/' + save_frame.format(img_name).replace('.{}'.format(img_type), '.npy'), rect_info, allow_pickle=True)
        return
    else:
        return img, nose_find


#################################################
if __name__ == '__main__':
    os.chdir('RHouse/Proctoring')
    face_model = get_face_detector()
    landmark_model = get_landmark_model()
    # left = [36, 37, 38, 39, 40, 41]
    # right = [42, 43, 44, 45, 46, 47]
    # nose_label = [28, 29, 30, 31, 32, 33, 34, 35]
    nose_label = [30]  #nosetip

    # test img
    pic_dir = 'pic_test/'
    for file in os.listdir(pic_dir):
        if file.endswith('.jpg') or file.endswith('.png') or file.endswith('.jpeg'):
            img_name = file
            img = cv2.imread('{}{}'.format(pic_dir, img_name))
            img_type = img_name.split('.')[-1]
            img_nose_label(img, img_name, face_model, nose_point=nose_label, save=True, img_type=img_type, save_frame='test_nosetip_{}')



