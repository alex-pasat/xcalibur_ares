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
xBnds = [0,5000]
yBnds = [500,1500]
w = xBnds[1]-xBnds[0]
h = yBnds[1]-yBnds[0]
knife = image[yBnds[0]:yBnds[1], xBnds[0]:xBnds[1]]
if DEBUG: dispImage(knife)

mask = np.zeros(knife.shape[:2],np.uint8)
bgdModel = np.zeros((1,65),np.float64)
fgdModel = np.zeros((1,65),np.float64)
rect = (0,0,h,w)
cv2.grabCut(knife,mask,rect,bgdModel,fgdModel,5,cv2.GC_INIT_WITH_RECT)
mask2 = np.where((mask==2)|(mask==0),0,1).astype('uint8')
knife = knife*mask2[:,:,np.newaxis]
dispImage(knife)

