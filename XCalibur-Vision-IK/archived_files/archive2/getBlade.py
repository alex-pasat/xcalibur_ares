import cv2
import numpy as np
import pyautogui
import os

def getFilenames(root_dir):
    relative_paths = []

    for _, _, filenames in os.walk(root_dir):
        for filename in filenames:
            relative_paths.append(filename)

    return relative_paths

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

def fwd_avg(vals, n):
    if n <= 0:
        raise ValueError("n must be positive")
    if n > len(vals):
        return np.array([])

    kernel = np.ones(n) / n
    return np.convolve(vals, kernel, mode='valid')

def getBladeContour(hsv, debug=False):
    # get dimensions
    h, w = hsv.shape[:2]

    # Mask to only take blade coloured pixels
    # grey_lwr_thr = np.array([0, 0, 100])
    # grey_upr_thr = np.array([180, 65, 255])
    grey_lwr_thr = np.array([0, 0, 140])
    grey_upr_thr = np.array([140, 30, 255])
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

    if debug: dispContour(img, blade_contour, "largest contour")

    # Take upper half of contour
    top_contour = []

    for x in range(w):
        slice = np.where(blade_contour[:, 0, 0] == x)[0]
        if slice.size > 0:
            top_pt = slice[np.argmin(blade_contour[slice, 0, 1])]
            top_contour.append(blade_contour[top_pt:top_pt+1, 0:1, :])  # shape (1,1,2)

    top_contour = np.concatenate(top_contour, axis=0).astype(np.int32) # shape (N,1,2)
    top_contour = top_contour[np.argsort(top_contour[:, 0, 0])] # sort by ascending x values

    if debug: dispContour(img, top_contour, "top contour")

    # Find blade hilt as first point to right of clamp with downward slope moving avg more than threshold
    slope_threshold = 0.4
    slope_samples = 50
    clampRight = 2000  # approximate coordinate of right end of clamp

    x_full = top_contour[:, 0, 0].astype(np.float32)
    y_full = top_contour[:, 0, 1].astype(np.float32)
    right_mask = x_full >= clampRight
    x_right = x_full[right_mask] 
    y_right = y_full[right_mask]

    dx = np.diff(x_right)
    dy = np.diff(y_right)
    slopes = dy / (dx + 1e-6) # avoids div by zero
    avg_slopes = fwd_avg(slopes, slope_samples)

    hiltX = x_right[np.where(avg_slopes > slope_threshold)[0][0]]

    # Now split the contour based on the hilt and get the blade profile
    hiltMask = top_contour[:,0,0] <= hiltX
    blade_profile = top_contour[hiltMask]
    blade_profile = blade_profile.astype(np.int32)

    if debug: dispContour(img, blade_profile, "blade profile")

    return blade_profile

def loadHSV(path):
    img = cv2.imread(path)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    return hsv, img

# images = getFilenames(r"C:\Users\h2par\OneDrive\Desktop\School\Terms\4B\MTE 482\Vision\rpiImages")

# for file in images:
#     print(file)
#     hsv,img = loadHSV("../rpiImages/" + file)
#     blade_profile = getBladeContour(hsv, debug=True)
#     dispContour(img, blade_profile, file)

hsv,img = loadHSV("../rpiImages/topLight.jpg")
blade_profile = getBladeContour(hsv, debug=True)
dispContour(img, blade_profile, "blade profile")

# np.save("blade_profile.npy", blade_profile)
