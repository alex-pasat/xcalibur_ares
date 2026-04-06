import cv2
import numpy as np
import matplotlib.pyplot as plt
import pyautogui

def dispImage(image):
    screen_width, screen_height = pyautogui.size()
    h, w = image.shape[:2]
    factor = min(0.9*screen_width / w, 0.9*screen_height / h)
    new_w = max(1, int(w * factor))
    new_h = max(1, int(h * factor))
    cv2.imshow("", cv2.resize(image, (new_w, new_h)))
    cv2.waitKey(0)

# ---- Load image ----
image_path = "../imagesWithTarget/knife1.jpeg"
img = cv2.imread(image_path)
original = img.copy()

# crop to only include blade
# xBnds = [0,5000]
# yBnds = [500,3500]
xBnds = [0,5000]
yBnds = [500,1500]
w = xBnds[1]-xBnds[0]
h = yBnds[1]-yBnds[0]
knife = img[yBnds[0]:yBnds[1], xBnds[0]:xBnds[1]]
dispImage(knife)

def process(img):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_canny = cv2.Canny(img_gray, 20, 100, apertureSize=3, L2gradient=True)
    dispImage(img_canny)
    kernel = np.ones((3, 3))
    img_dilate = cv2.dilate(img_canny, kernel, iterations=3)
    dispImage(img_dilate)

    contours, _ = cv2.findContours(img_dilate,
                               cv2.RETR_EXTERNAL,
                               cv2.CHAIN_APPROX_NONE)

    clean = np.zeros_like(img_dilate)
    cv2.drawContours(img, contours, -1, (0, 0, 255), thickness=cv2.FILLED)
    img_clean = clean
    dispImage(img)

    img_erode = cv2.erode(img_clean, kernel, iterations=10)
    dispImage(img_erode)
    return img_erode


contours, _ = cv2.findContours(process(knife), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
cv2.drawContours(knife, contours, -1, (0, 255, 0), 2)
dispImage(knife)
