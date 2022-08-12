import os
import cv2
import numpy as np


def grid_video(f, video_route, img_max=-1, grid_interval=15):
    if not os.path.isfile(video_route + f):
        print('Cannot find {}'.format(video_route + f))
        return
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
    if img_max < 0:
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
        # print("\rprocess: {}/{}".format(idx + 1, frame_count), end='')
        if n >= img_max:
            break
    vc.release()
    print('total img: {}'.format(n))
    

########################################
if __name__ == '__main__':
    video_route = 'scripts/video/'
    f = 'May_Leigh_RGB_trial.mp4'
    grid_video(f, video_route, 123, 10)
