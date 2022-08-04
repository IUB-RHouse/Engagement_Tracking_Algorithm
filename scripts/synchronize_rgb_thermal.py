'''
Apply the location information got from rgb image on thermal image
'''
import os
import cv2
import numpy as np
from nose_tracker_on_rgb_img import *
from grid_video_to_img import grid_video



class NoseTracking():
    def __init__(self, file_title='06_07_2022-14_23_27', main_dir='RHouse/Proctoring/', coor_dict_path='RHouse/video/'):
        # self.nose_label = [28, 29, 30, 31, 32, 33, 34, 35]
        self.nose_label = [30]
        self.video_img_dir = r'D://Andrew/U/IUB_MSDS/pythonProject/RHouse/video/{}/'.format(file_title)
        self.file_title = file_title
        self.rgb_file_name = '{}_RGB_grid/'.format(self.file_title)
        self.thermal_file_name = '{}_THERMAL_grid/'.format(self.file_title)
        self.main_dir = main_dir
        self.face_model = get_face_detector(main_dir=main_dir)
        self.landmark_model = get_landmark_model(self.main_dir + 'models/pose_model')
        self.pic_dir = self.video_img_dir + self.rgb_file_name
        self.coor_dict = np.load(coor_dict_path + 'thermal_cam_coordinate.npy', allow_pickle=True).item()

    def grid_video_to_img(self, max_pic_n=30, visual_interval=15, video_form='{}_RGB.avi'):
        # Visual Video
        f = video_form.format(self.file_title)
        rgb_frame_count = grid_video(f, self.video_img_dir, max_pic_n, visual_interval)
        # Thermal Video
        f = video_form.format(self.file_title).replace('RGB', 'THERMAL')
        thermal_frame_count = grid_video(f,self.video_img_dir , max_pic_n, visual_interval * 2, default_frame_count= rgb_frame_count * 2)

    def nose_detect_on_rgb_img(self):
        for file in os.listdir(self.pic_dir):
            if file.endswith('.jpg') or file.endswith('.png'):
                img_name = file
                img = cv2.imread('{}{}'.format(self.pic_dir, img_name))
                # img_nose_label(img, img_name, self.face_model, self.landmark_model, nose_point=self.nose_label, save=True, save_dir=self.pic_dir + 'test/', save_frame='test_nosetip_{}')
                img_nose_label_cropped(self.coor_dict, img, img_name, self.face_model, self.landmark_model, nose_point=self.nose_label, save=True, save_dir=self.pic_dir + 'test/', save_frame='test_nosetip_{}')

    # def find_response_area_on_thermal(self, rect_info_dir, file, thermal_img_name, height_adj_para=None):
    #     """
    #     We can abandon this function....
    #     """
    #     rect_info = np.load(rect_info_dir + file, allow_pickle=True).item()
    #     thermal_img = cv2.imread(self.video_img_dir + self.thermal_file_name + thermal_img_name)
    #     x1, x2 = rect_info['rect'][0], rect_info['rect'][2]
    #     y1, y2 = rect_info['rect'][1], rect_info['rect'][3]
    #     w1, w2 = 0, len(thermal_img[0])
    #     h1, h2 = 0, len(thermal_img)
    #     # ki, kj = 0, 0
    #     k_left = (min(rect_info['nose_area'][1]) - x1) * w2 / (x2 - x1)
    #     k_right = (max(rect_info['nose_area'][1]) - x1) * w2 / (x2 - x1)
    #     # thermal_img[:, round(k_left)] = [0, 0, 0]
    #     # thermal_img[:, round(k_right)] = [0, 0, 0]
    #     scale_para = (w2 - w1) / (x2 - x1)
    #     visual_nose_length = max(rect_info['nose_area'][0]) - min(rect_info['nose_area'][0])
    #     nose_length_after_scale = round(visual_nose_length * scale_para)
    #     if height_adj_para is None:
    #         try_make_dir(self.video_img_dir + self.thermal_file_name + 'test1/')
    #         m = round(h2 // 3)
    #         goal_m = m
    #         min_color = 255
    #         while m < h2 - nose_length_after_scale:
    #             mean_color = np.mean(thermal_img[m:m + nose_length_after_scale, round(k_left): round(k_right)])
    #             # print(m, mean_color)
    #             if mean_color < min_color:
    #                 min_color = mean_color
    #                 goal_m = m
    #             m += 10
    #         thermal_img_tmp = thermal_img.copy()
    #         thermal_img_tmp[goal_m, round(k_left): round(k_right)] = [0, 0, 0]
    #         thermal_img_tmp[goal_m + nose_length_after_scale, round(k_left): round(k_right)] = [0, 0, 0]
    #         thermal_img_tmp[goal_m:goal_m + nose_length_after_scale, round(k_left)] = [0, 0, 0]
    #         thermal_img_tmp[goal_m:goal_m + nose_length_after_scale, round(k_right)] = [0, 0, 0]
    #         cv2.imwrite(self.video_img_dir + self.thermal_file_name + 'test1/m{}_'.format(m) + thermal_img_name, thermal_img_tmp)

    def find_response_area_on_thermal_v2(self, rect_info_dir, file, thermal_img_name, label_nosetip=True):
        """
        Apply the values get from thermal_cam_relocate.py
        :param label_nosetip: True if you want to see where is the nosetip pixel and label it on thermal image.
        """
        rect_info = np.load(rect_info_dir + file, allow_pickle=True).item()
        thermal_img = cv2.imread(self.video_img_dir + self.thermal_file_name + thermal_img_name)
        y1, x1, y2, x2 = self.coor_dict['upper'], self.coor_dict['left'], self.coor_dict['lower'], self.coor_dict['right']

        w1, w2 = 0, len(thermal_img[0])
        h1, h2 = 0, len(thermal_img)

        nosetip_x = round((rect_info['nosetip_point'][0] - x1) * w2 / (x2 - x1))
        nosetip_y = round((rect_info['nosetip_point'][1] - y1) * w2 / (x2 - x1))
        if (nosetip_y >= h2) or (nosetip_x >= w2) or (nosetip_y < 0) or (nosetip_x < 0):
            # detected nosetip point is out of thermal camera
            nosetip_pixel = None
        else:
            nosetip_pixel = thermal_img[nosetip_y, nosetip_x][0]
            # pixel_at_nose += img[5][5]  # replace with the pixels at the nosetip
            # if taking an average, then save the pixels in the different regions

        check_point = thermal_img[0, 0][0]  # a point in the top corner (black point) - that does not omit thermal heat
        if label_nosetip:
            for i in range(-10, 10):
                try:
                    thermal_img[nosetip_y + i, nosetip_x + i] = [255, 255, 255]
                    thermal_img[nosetip_y + i, nosetip_x - i] = [255, 255, 255]
                except:
                    pass
        cv2.imwrite(self.video_img_dir + self.thermal_file_name + 'test/' + thermal_img_name, thermal_img)

        np.save(self.video_img_dir + self.thermal_file_name + 'test/' + 'nosetip_point_{}.npy'.format(
            thermal_img_name.split('.')[0]), {'nosetip_x': nosetip_x, 'nosetip_y': nosetip_y, 'pixel_value': nosetip_pixel, 'check_point': check_point}, allow_pickle=True)
        # need to identify a point in the top corner (black point) - that does not omit thermal heat
        # find the temperature at that point
        # repeat the steps above.

    def apply_on_thermal_img(self):
        rect_info_dir = self.pic_dir + 'test/rectangle_info/'
        try_make_dir(dir1=self.video_img_dir + self.thermal_file_name + 'test/')
        for file in os.listdir(rect_info_dir):
            id = file[:-4].split('-')[-1]
            thermal_id = str(int(id) * 2).zfill(len(id))
            thermal_img_name = self.file_title + '_THERMAL-' + thermal_id + '.jpg'
            if not os.path.isfile(self.video_img_dir + self.thermal_file_name + thermal_img_name):
                print('Cannot find {}'.format(thermal_img_name))
                break
            self.find_response_area_on_thermal_v2(rect_info_dir, file, thermal_img_name)

    def time_series_nosetip_pixel(self):
        ts_record = {}
        for file in os.listdir(self.video_img_dir + self.thermal_file_name + 'test/'):
            if file.endswith('.npy'):
                record_i = np.load(self.video_img_dir + self.thermal_file_name + 'test/' + file, allow_pickle=True).item()
                ts_record[int(file.split('-')[1][:-4])] = record_i
        ts_df = pd.DataFrame.from_dict(ts_record).transpose()
        return ts_df
        # df = pd.DataFrame.from_dict(my_dict)

        # save as json file

        # dict --> json
        # transform data when using
        # pd.read_json(j0, orient='index')

    def main(self,  video_form='MS_test2_RGB.avi'):
        tstart = time.time()
        self.grid_video_to_img(max_pic_n=-1, visual_interval=1, video_form=video_form)
        self.nose_detect_on_rgb_img()
        self.apply_on_thermal_img()
        pixel_data = self.time_series_nosetip_pixel()
        pixel_data['thermal_frame_index'] = pixel_data.index
        print('usage time=', time.time() - tstart)
        print(pixel_data.head())
        pixel_data.to_csv(self.video_img_dir + self.thermal_file_name + 'test/pixel_df.csv', line_terminator='\n')
        return pixel_data


#########################################################
#########################################################
if __name__ == '__main__':
    NT = NoseTracking(file_title='MS_rps1', main_dir='RHouse/Proctoring/', coor_dict_path='RHouse/video/')
    pixel_df = NT.main(video_form='MS_rps1_RGB.avi')
