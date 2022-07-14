'''
Apply the location information got from rgb image on thermal image
'''
import os
import cv2
import numpy as np
from nose_tracker_on_rgb_img import *
from grid_video_to_img import grid_video


class NoseTracking():
    def __init__(self, file_title = '06_07_2022-14_23_27'):
        self.nose_label = [28, 29, 30, 31, 32, 33, 34, 35]
        self.video_img_dir = 'video/Salma_Daughter/'
        self.file_title = file_title
        self.rgb_file_name = '{}_RGB_grid/'.format(self.file_title)
        self.thermal_file_name = '{}_THERMAL_grid/'.format(self.file_title)
        self.face_model = get_face_detector()
        self.landmark_model = get_landmark_model()
        self.pic_dir = self.video_img_dir + self.rgb_file_name

    def grid_video_to_img(self, max_pic_n=30, visual_interval=10):
        # Visual Video
        f = '06_07_2022-14_23_27_RGB.avi'
        grid_video(f, self.video_img_dir , max_pic_n, visual_interval)
        # Thermal Video
        f = '06_07_2022-14_23_27_THERMAL.avi'
        grid_video(f,self.video_img_dir , max_pic_n, visual_interval * 2)

    def nose_detect_on_rgb_img(self):
        for file in os.listdir(self.pic_dir):
            if file.endswith('.jpg') or file.endswith('.png'):
                img_name = file
                img = cv2.imread('{}{}'.format(self.pic_dir, img_name))
                img_nose_label(img, img_name, self.face_model, nose_point=self.nose_label, save=True, save_dir=self.pic_dir + 'test/')

    def find_response_area_on_thermal(self, rect_info_dir, file, thermal_img_name, height_adj_para=None):
        rect_info = np.load(rect_info_dir + file, allow_pickle=True).item()
        thermal_img = cv2.imread(self.video_img_dir + self.thermal_file_name + thermal_img_name)
        x1, x2 = rect_info['rect'][0], rect_info['rect'][2]
        y1, y2 = rect_info['rect'][1], rect_info['rect'][3]
        w1, w2 = 0, len(thermal_img[0])
        h1, h2 = 0, len(thermal_img)
        # ki, kj = 0, 0
        k_left = (min(rect_info['nose_area'][1]) - x1) * w2 / (x2 - x1)
        k_right = (max(rect_info['nose_area'][1]) - x1) * w2 / (x2 - x1)
        # thermal_img[:, round(k_left)] = [0, 0, 0]
        # thermal_img[:, round(k_right)] = [0, 0, 0]
        scale_para = (w2 - w1) / (x2 - x1)
        visual_nose_length = max(rect_info['nose_area'][0]) - min(rect_info['nose_area'][0])
        nose_length_after_scale = round(visual_nose_length * scale_para)
        if height_adj_para is None:
            try_make_dir(self.video_img_dir + self.thermal_file_name + 'test1/')
            m = round(h2 // 3)
            goal_m = m
            min_color = 255
            while m < h2 - nose_length_after_scale:
                mean_color = np.mean(thermal_img[m:m + nose_length_after_scale, round(k_left): round(k_right)])
                # print(m, mean_color)
                if mean_color < min_color:
                    min_color = mean_color
                    goal_m = m
                m += 10
            thermal_img_tmp = thermal_img.copy()
            thermal_img_tmp[goal_m, round(k_left): round(k_right)] = [0, 0, 0]
            thermal_img_tmp[goal_m + nose_length_after_scale, round(k_left): round(k_right)] = [0, 0, 0]
            thermal_img_tmp[goal_m:goal_m + nose_length_after_scale, round(k_left)] = [0, 0, 0]
            thermal_img_tmp[goal_m:goal_m + nose_length_after_scale, round(k_right)] = [0, 0, 0]
            cv2.imwrite(self.video_img_dir + self.thermal_file_name + 'test1/m{}_'.format(m) + thermal_img_name, thermal_img_tmp)

    def apply_on_thermal_img(self):
        rect_info_dir = self.pic_dir + 'test/rectangle_info/'
        height_adj_para = None
        try_make_dir(dir1=self.video_img_dir + self.thermal_file_name + 'test/')
        for file in os.listdir(rect_info_dir):
            id = file[:-4].split('-')[-1]
            thermal_id = str(int(id) * 2).zfill(len(id))
            thermal_img_name = self.file_title + '_THERMAL-' + thermal_id + '.jpg'
            if not os.path.isfile(self.video_img_dir + self.thermal_file_name + thermal_img_name):
                print('Cannot find {}'.format(thermal_img_name))
                break
            self.find_response_area_on_thermal(rect_info_dir, file, thermal_img_name, height_adj_para)


#########################################################
#########################################################
if __name__ == '__main__':
    NT = NoseTracking()
    NT.grid_video_to_img()
    NT.nose_detect_on_rgb_img()
    NT.apply_on_thermal_img()