# Nose Tip Tracking Task

## Approach
1. Grid Video (*grid_video.py*)
    1. Extract frame images from visual video
    2. Extract frame images from thermal video
3. Find Nose Tip location on image (*nose_tracker_on_rgb+img.py*)
    1. load dlib trained models(*face_landmark.py*, *face_detector.py*)
    1. find face rectangle on rgb image
    2. find location of nose on rgb image
    3. mark the nose tip pixel and save the image
    4. save the nose tip position information and face rectangle information as *.npy* file
5. find response position on thermal image (*synchronize_rgb_thermal.py/ apply_on_thermal_img()*)
    1. load *coor_dict*, get frame information of thermal camera
    2. use the nose tip postion detected from rgb image, calculate respose position of nose tip
    3. label nose tip position on thermal image, save it
7. find pixel values (temperature of nose tip)
    1. record the value of pixel of nose tip position on thermal image
    2. combine as time-series dataframe
    3. save as *pixel_data.csv*



## Operate Steps

### Check Environment
1. This code is in Python 3.6
2. download *Engagement_Tracking_Algorithm* to local PC, unzip it
3. The codes are in *scripts*

### Download external files
Due to the limit of file size, some necessary documents cannot be uploaded to github. Therefore user has to download them externally.
1. see scripts/video/README.md
    1. download a set of videos (there are two videos for one set: visual video and thermal video). Ask Leigh if you need video files.
    2. create a folder named by the string of video name before the last underscore. For example, if the videos are "MS_test1_RGB.avi" and "MS_test2_THERMAL.avi", you should create a folder called * MS_test1/* that under *scripts/video/*
    3. store the two videos in the folder you just create.
3. see scripts/protoring/models/pose_model/pose_model/ReadMe.md
    1. Check if there are two files in folder variables: *variable.index* and *variables.data-00000-of-00001*
    2. If lack of *variables.data-00000-of-00001*, use the following link to download the file and store in folder *scripts/protoring/models/variables*: https://drive.google.com/file/d/18o0tIqrLWnoI0e-GxiC4xPlK4zfXLM7O/view?usp=sharing

### Operate
1. find *scripts/synchronize_rgb_thermal.py*
2. `python synchronize_rgb_thermal.py`
3. Enter video file title (for example: MS_test1), this is which video set you want to test
