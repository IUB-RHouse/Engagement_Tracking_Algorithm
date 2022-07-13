'''
Apply the location information got from rgb image on thermal image
'''
import os
import cv2
import numpy as np
from nose_tracker_on_rgb_img import *

##########################################################
# Use Salma's Daughter video (06/07/2022) as example file
video_img_dir = 'video/'
file_title = '06_07_2022-14_23_27'
rgb_file_name = '{}_RGB_grid/'.format(file_title)
thermal_file_name = '{}_THERMAL_grid/'.format(file_title)
##########################################################

face_model = get_face_detector()
landmark_model = get_landmark_model()
nose_label = [28, 29, 30, 31, 32, 33, 34, 35]

pic_dir = video_img_dir + rgb_file_name
for file in os.listdir(pic_dir):
    if file.endswith('.jpg') or file.endswith('.png'):
        img_name = file
        img = cv2.imread('{}{}'.format(pic_dir, img_name))
        img_nose_label(img, img_name, face_model, nose_point=nose_label, save=True, save_dir=pic_dir + 'test/')

rect_info_dir = pic_dir + 'test/rectangle_info/'
height_adj_para = None
try_make_dir(dir1=video_img_dir + thermal_file_name + 'test/')
for file in os.listdir(rect_info_dir):
    id = file[:-4].split('-')[-1]
    thermal_id = str(int(id) * 2).zfill(len(id))
    thermal_img_name = file_title + '_THERMAL-' + thermal_id + '.jpg'
    if not os.path.isfile(video_img_dir + thermal_file_name + thermal_img_name):
        print('Cannot find {}'.format(thermal_img_name))
        break
    rect_info = np.load(rect_info_dir + file, allow_pickle=True).item()
    thermal_img = cv2.imread(video_img_dir + thermal_file_name + thermal_img_name)
    x1, x2 = rect_info['rect'][0], rect_info['rect'][2]
    y1, y2 = rect_info['rect'][1], rect_info['rect'][3]
    w1, w2 = 0, len(thermal_img[0])
    h1, h2 = 0, len(thermal_img)
    # ki, kj = 0, 0
    len_w = len(rect_info['nose_area'][1])
    k_left = (min(rect_info['nose_area'][1]) - x1) * w2 / (x2 - x1)
    k_right = (max(rect_info['nose_area'][1]) - x1) * w2 / (x2 - x1)
    scale_para = (w2 - w1) / (x2 - x1)
    visual_nose_length = max(rect_info['nose_area'][0]) - min(rect_info['nose_area'][0])
    nose_length_after_scale = round(visual_nose_length * scale_para)
    if height_adj_para is None:
        try_make_dir(video_img_dir + thermal_file_name + 'test1/')
        m = round(h2 // 3)
        goal_m = m
        min_color = 255
        while m < h2 - nose_length_after_scale:
            mean_color = np.mean(thermal_img[m:m + nose_length_after_scale, round(k_left): round(k_right)])
            print(m, mean_color)
            if mean_color < min_color:
                min_color = mean_color
                goal_m = m
            m += 10
        thermal_img_tmp = thermal_img.copy()
        thermal_img_tmp[goal_m, round(k_left): round(k_right)] = [0, 0, 0]
        thermal_img_tmp[goal_m+ nose_length_after_scale, round(k_left): round(k_right)] = [0, 0, 0]
        thermal_img_tmp[goal_m:goal_m + nose_length_after_scale, round(k_left)] = [0, 0, 0]
        thermal_img_tmp[goal_m:goal_m + nose_length_after_scale, round(k_right)] = [0, 0, 0]
        cv2.imwrite(video_img_dir + thermal_file_name + 'test1/m{}_'.format(m) + thermal_img_name, thermal_img_tmp)
