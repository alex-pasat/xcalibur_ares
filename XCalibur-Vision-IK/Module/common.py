import cv2
#import pyautogui
import os
import numpy as np

def dispImage(image, text):
    screen_width, screen_height = pyautogui.size()
    h, w = image.shape[:2]
    factor = min(0.9*screen_width / w, 0.9*screen_height / h)
    new_w = max(1, int(w * factor))
    new_h = max(1, int(h * factor))
    cv2.imshow(text, cv2.resize(image, (new_w, new_h)))
    cv2.waitKey(0)

def dispContour(img, contour, text, isClosed=False):
    output = img.copy()
    if contour is not None:
        cv2.polylines(output, [contour], isClosed=isClosed, color=(0, 255, 0), thickness=10)
    dispImage(output, text)

def getFilenames(root_dir):
    relative_paths = []

    for _, _, filenames in os.walk(root_dir):
        for filename in filenames:
            relative_paths.append(filename)

    return relative_paths

def flipZ(normals):
    result = []
    for i in range(len(normals)):
        n = np.array([normals[i][0],normals[i][1],-normals[i][2]])
        result.append(n)
    return result
