import os
import cv2
import numpy as np


golden_corner = (705, 995, 850, 1177)
'''
the four values reponse to upper, left, lower, right
It is the inference "rectangle" of thermal camera on rgb image
'''
video_img_dir = 'video/video_1/'
rgb_file_name = '{}_RGB_grid/'.format(file_title)
thermal_file_name = '{}_THERMAL_grid/'.format(file_title)
relocate_rgb_dir = video_img_dir + rgb_file_name + 'relocate_task/'
relocate_thermal_dir = video_img_dir + thermal_file_name + 'relocate_task/'

pic_dir = video_img_dir + rgb_file_name
rect_info_dir = pic_dir + 'test/rectangle_info/'

ftp_diff = 2

from nose_tracker_on_rgb_img import try_make_dir
try_make_dir(dir1=video_img_dir + thermal_file_name + 'test2/')  # store in "
for file in os.listdir(rect_info_dir):
    if not os.path.isfile(pic_dir + 'test/' + file.replace('.npy', '.jpg')):
        continue
    id = file[:-4].split('-')[-1]
    thermal_id = str(int(id) * ftp_diff).zfill(len(id))
    thermal_img_name = file_title + '_THERMAL-' + thermal_id + '.jpg'
    if not os.path.isfile(video_img_dir + thermal_file_name + thermal_img_name):
        print('Cannot find {}'.format(thermal_img_name))
        break

    rect_info = np.load(rect_info_dir + file, allow_pickle=True).item()
    thermal_img = cv2.imread(video_img_dir + thermal_file_name + thermal_img_name)
    nose_rect = [min(rect_info['nose_area'][0]), min(rect_info['nose_area'][1]), max(rect_info['nose_area'][0]), max(rect_info['nose_area'][1])]
    transfor_nose_rect = [(nose_rect[0] - possi_upper) * scale_para]
    transfor_nose_rect.append((nose_rect[1] - possi_lefter) * scale_para)
    transfor_nose_rect.append(len(thermal_img) - (possi_lower - nose_rect[2]) * scale_para)
    transfor_nose_rect.append(len(thermal_img[0]) - (possi_righter - nose_rect[3]) * scale_para)
    transfor_nose_rect = [round(c) for c in transfor_nose_rect]
    transfor_nose_rect = [max(c, 0) for c in transfor_nose_rect]
    transfor_nose_rect = [max(transfor_nose_rect[0], 0), max(transfor_nose_rect[1], 0),
                          min(transfor_nose_rect[2], len(thermal_img) - 1),
                          min(transfor_nose_rect[3], len(thermal_img[0]) - 1)]
    thermal_img_tmp = thermal_img.copy()
    thermal_img_tmp[transfor_nose_rect[0], transfor_nose_rect[1]: transfor_nose_rect[3]] = [0, 0, 0]
    thermal_img_tmp[transfor_nose_rect[2], transfor_nose_rect[1]: transfor_nose_rect[3]] = [0, 0, 0]
    thermal_img_tmp[transfor_nose_rect[0]: transfor_nose_rect[2], transfor_nose_rect[1]] = [0, 0, 0]
    thermal_img_tmp[transfor_nose_rect[0]: transfor_nose_rect[2], transfor_nose_rect[3]] = [0, 0, 0]
    cv2.imwrite(video_img_dir + thermal_file_name + 'test2/' + thermal_img_name, thermal_img_tmp)
