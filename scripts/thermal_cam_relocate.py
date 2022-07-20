import os
import cv2
import numpy as np


def get_manually_label_location(img):
    label_point = []
    for i in range(len(img)):
        for j in range(len(img[0])):
            if np.mean(img[i, j]) == 0:
                label_point.append((i, j))
    label_record = {'left': [], 'right': []}
    label_point.sort(key=lambda x: x[1])
    label_record['left'].append(label_point[0])
    flag = 'left'
    for k in range(1, len(label_point)):
        if label_point[k][1] > label_point[k - 1][1] + 1:
            flag = 'right'
        label_record[flag].append(label_point[k])

    left_center = (
    round(np.mean([h[0] for h in label_record['left']])), round(np.mean([h[1] for h in label_record['left']])))
    right_center = (
    round(np.mean([h[0] for h in label_record['right']])), round(np.mean([h[1] for h in label_record['right']])))
    return (left_center, right_center)


#################################################################################################################
def calculate_thermal_cam_coordinate_on_rgb_img(rgb_manual_center, thermal_manual_center):
    '''
    x, y for rgb img
    w, h for thermal img
    output: (possi_upper, possi_lefter, possi_lower, possi_righter): coordinates on rgb img
    '''
    x1, x2 = rgb_manual_center[0][1], rgb_manual_center[1][1]
    x_eyelen = x2 - x1
    y1, y2 = rgb_manual_center[0][0], rgb_manual_center[1][0]
    w1, w2 = thermal_manual_center[0][1], thermal_manual_center[1][1]
    h1, h2 = thermal_manual_center[0][0], thermal_manual_center[1][0]
    w_eyelen = w2 - w1
    scale_para = (w2 - w1) / (x2 - x1)  # the enlarge scale from rgb to thermal

    h1_w_ratio = h1 / w_eyelen
    h2_w_ratio = h2 / w_eyelen
    possi_upper = round(np.mean([y1 - x_eyelen * h1_w_ratio, y2 - x_eyelen * h2_w_ratio]))
    possi_lower = round(possi_upper + (len(thermal_mask_img) / w_eyelen) * x_eyelen)

    possi_lefter = round(np.mean([x1 - x_eyelen * (w1 / w_eyelen), x2 - x_eyelen * (w2 / w_eyelen)]))
    possi_righter = round(possi_lefter + len(thermal_mask_img[0]) / scale_para)
    print('simulate height/width of thermal camera=', (possi_lower - possi_upper) / (possi_righter - possi_lefter))
    print('True height/width of thermal camera=', len(thermal_mask_img) / len(thermal_mask_img[0]))
    return (possi_upper, possi_lefter, possi_lower, possi_righter)


#################################################################################################################
def test_relocate_image(relocate_rgb_dir, possi_coor, file = '06_07_2022-14_23_27_RGB-00000040.jpg'):
    """
    Draw the thermal camera range on rgb image
    """
    possi_upper, possi_lefter, possi_lower, possi_righter = possi_coor[0], possi_coor[1], possi_coor[2], possi_coor[3]
    img = cv2.imread(relocate_rgb_dir + file)
    img[possi_upper, possi_lefter:possi_righter] = [0, 0, 0]
    img[possi_lower, possi_lefter:possi_righter] = [0, 0, 0]
    img[possi_upper:possi_lower, possi_lefter] = [0, 0, 0]
    img[possi_upper:possi_lower, possi_righter] = [0, 0, 0]
    cv2.imwrite(relocate_rgb_dir + file[:-4] + '_relocate.jpg', img)


#################################################################################################################
file_title = '06_07_2022-14_23_27'
video_img_dir = 'video/06_07_2022-14_23_27/'  # where video stored
rgb_file_name = '{}_RGB_grid/'.format(file_title)
thermal_file_name = '{}_THERMAL_grid/'.format(file_title)
relocate_rgb_dir = video_img_dir + rgb_file_name + 'relocate_task/'
relocate_thermal_dir = video_img_dir + thermal_file_name + 'relocate_task/'

'''
'06_07_2022-14_23_27_RGB-00000130_mask.jpg' and '06_07_2022-14_23_27_THERMAL-00000260_mask.jpg' are manually colored images based on '06_07_2022-14_23_27_RGB-00000130_mask.jpg' and '06_07_2022-14_23_27_THERMAL-00000260.jpg'
'06_07_2022-14_23_27_RGB-00000130_mask.jpg' is all white with only two black rectangles, which indicate the locations of facial features we are easily to locate (for example, two eyes).
So is '06_07_2022-14_23_27_THERMAL-00000260_mask.jpg'.
'''
rgb_mask = '06_07_2022-14_23_27_RGB-00000130_mask.jpg'
thermal_mask = '06_07_2022-14_23_27_THERMAL-00000260_mask.jpg'
print('File exist?'. os.path.isfile(relocate_rgb_dir + rgb_mask))
rgb_mask_img = cv2.imread(relocate_rgb_dir + rgb_mask)
thermal_mask_img = cv2.imread(relocate_thermal_dir + thermal_mask)

rgb_manual_center = get_manually_label_location(rgb_mask_img)
thermal_manual_center = get_manually_label_location(thermal_mask_img)

thermal_cam_coor = calculate_thermal_cam_coordinate_on_rgb_img(rgb_manual_center, thermal_manual_center)
print(thermal_cam_coor)
# (697, 984, 862, 1190)
np.save(video_img_dir + 'thermal_cam_coordinate.npy', {'upper': thermal_cam_coor[0], 'left': thermal_cam_coor[1], 'lower': thermal_cam_coor[2],
         'right': thermal_cam_coor[3]}, allow_pickle=True)

test_image = '06_07_2022-14_23_27_RGB-00000040.jpg'
test_relocate_image(relocate_rgb_dir, possi_coor=thermal_cam_coor, file=test_image)
