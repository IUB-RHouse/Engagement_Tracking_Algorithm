'''
Original Reference Source: https://towardsdatascience.com/automating-online-proctoring-using-ai-e429086743c8
'''
import cv2
import numpy as np
import os

##########################################################################
def get_face_detector(modelFile=None, configFile=None, quantized=False, model_dir='scripts/protoring/models/'):
    if quantized:
        if modelFile == None:
            modelFile = "{}opencv_face_detector_uint8.pb".format(model_dir)
        if configFile == None:
            configFile = "{}opencv_face_detector.pbtxt".format(model_dir)
        model = cv2.dnn.readNetFromTensorflow(modelFile, configFile)
    else:
        if modelFile == None:
            modelFile = "{}res10_300x300_ssd_iter_140000.caffemodel".format(model_dir)
        if configFile == None:
            configFile = "{}deploy.prototxt".format(model_dir)
        if not os.path.isfile(configFile):
            print('Cannot find {}; May need to change \'model_dir\' in \'face_detector.py > get_face_detector()\''.format("{}deploy.prototxt".format(model_dir)))
        model = cv2.dnn.readNetFromCaffe(configFile, modelFile)
    return model

 
def find_faces(img, model):
    """
    Find the faces in an image   
    ----------
    img : np.uint8
        Image to find faces from
    model : dnn_Net
        Face detection model

    Returns
    -------
    faces : list
        List of coordinates of the faces detected in the image
    """
    h, w = img.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(img, (300, 300)), 1.0,
	(300, 300), (104.0, 177.0, 123.0))
    model.setInput(blob)
    res = model.forward()
    faces = []
    for i in range(res.shape[2]):
        confidence = res[0, 0, i, 2]
        if confidence > 0.5:
            box = res[0, 0, i, 3:7] * np.array([w, h, w, h])
            (x, y, x1, y1) = box.astype("int")
            faces.append([x, y, x1, y1])
    return faces

def draw_faces(img, faces):
    """
    Draw faces on image

    Parameters
    ----------
    img : np.uint8
        Image to draw faces on
    faces : List of face coordinates
        Coordinates of faces to draw

    Returns
    -------
    None.

    """
    for x, y, x1, y1 in faces:
        cv2.rectangle(img, (x, y), (x1, y1), (0, 0, 255), 3)
