'''
G
'''
import os
import cv2
import numpy as np


def grid_video(f, video_route, img_max=None, grid_interval=10):
    vc = cv2.VideoCapture(video_route + f)
    fps = vc.get(cv2.CAP_PROP_FPS)
    frame_count = int(vc.get(cv2.CAP_PROP_FRAME_COUNT))
    print(frame_count)
    video = []
    grid_file = video_route + '{}_grid/'.format(f[:-4])
    try:
        os.mkdir(grid_file)
    except:
        pass
    if img_max is None:
        img_max = np.Inf
    n = 0
    for idx in range(frame_count):
        if idx % grid_interval != 0:
            continue
        vc.set(1, idx)
        ret, frame = vc.read()
        height, width, layers = frame.shape
        size = (width, height)

        if frame is not None:
            file_name = '{}{}-{:08d}.jpg'.format(grid_file, f[:-4], idx)
            cv2.imwrite(file_name, frame)
            n += 1
        if n >= img_max:
            break
    vc.release()
    print('total img: {}'.format(len(os.listdir(grid_file))))
    

########################################
if __name__ == '__main__':
    video_route = 'video/'
    f = 'May_Leigh_RGB_trial.mp4'
    grid_video(f, video_route, 123, 10)
