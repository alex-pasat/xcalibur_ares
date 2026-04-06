import cv2
import numpy as np
import pyautogui
import os

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

hsv,img = cv2.imread("../imagesWithTarget/knife3.jpeg")

# get dimensions
h, w = hsv.shape[:2]
# Mask to only take blade coloured pixels
grey_lwr_thr = np.array([0, 0, 100])
grey_upr_thr = np.array([180, 65, 255])
mask = cv2.inRange(hsv, grey_lwr_thr, grey_upr_thr)

if debug: dispContour(mask, None, "original mask")
# Denoise mask by morphological closing and opening
kernel = np.ones((5,5), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
if debug: dispContour(mask, None, "closed and opened mask")
# Extract largest contour of mask to enclose the knife and optionally display
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
blade_contour = max(contours, key=cv2.contourArea)

