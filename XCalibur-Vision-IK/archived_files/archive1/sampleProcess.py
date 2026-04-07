import cv2
import numpy as np
import pyautogui

def dispImage(image):
    screen_width, screen_height = pyautogui.size()
    h, w = image.shape[:2]
    factor = min(0.9*screen_width / w, 0.9*screen_height / h)
    new_w = max(1, int(w * factor))
    new_h = max(1, int(h * factor))
    cv2.imshow("original image", cv2.resize(image, (new_w, new_h)))
    cv2.waitKey(0)
    

# set a debug flag optionally
DEBUG = True

# loading an image as a sample
image = cv2.imread('../imagesWithTarget/knife1.jpeg')
h, w = image.shape[:2]
# if DEBUG: dispImage(image)

# crop to only include blade
# xBnds = [0,5000]
# yBnds = [500,3500]
xBnds = [0,5000]
yBnds = [500,1500]
w = xBnds[1]-xBnds[0]
h = yBnds[1]-yBnds[0]
knife = image[yBnds[0]:yBnds[1], xBnds[0]:xBnds[1]]
if DEBUG: dispImage(knife)

# Create a mask for strokes in grabcut in a rectangle
# (I think there will be a rectangle where we are sure foreground will be)
strokeMask = np.zeros((h,w),np.uint8)
strokeMask[650:850, 2500:3800] = cv2.GC_FGD
strokeMask[strokeMask == cv2.GC_BGD] = cv2.GC_PR_FGD
strokeMask[950:2000, 2500:3800] = cv2.GC_BGD

# if DEBUG:
#     overlay = knife.copy()
#     color_layer = np.zeros_like(knife)
#     color_layer[strokeMask == cv2.GC_FGD] = (0, 255, 0)
#     color_layer[strokeMask == cv2.GC_BGD] = (0, 0, 255)
#     alpha = 0.5
#     overlay = cv2.addWeighted(overlay, 1-alpha, color_layer, alpha, 0)
#     dispImage(overlay)

bgdModel = np.zeros((1,65),np.float64)
fgdModel = np.zeros((1,65),np.float64)
mask, bgdModel, fgdModel = cv2.grabCut(knife,strokeMask,None,bgdModel,fgdModel,1,cv2.GC_INIT_WITH_MASK)
mask = np.where((mask==2)|(mask==0),0,1).astype('uint8')
knife = knife*mask[:,:,np.newaxis]
if DEBUG: dispImage(knife)
