import cv2
import numpy as np
import pyautogui

def dispImage(image, window_name="Image"):
    """Display image scaled to screen size"""
    screen_width, screen_height = pyautogui.size()
    h, w = image.shape[:2]
    factor = min(0.9*screen_width / w, 0.9*screen_height / h)
    new_w = max(1, int(w * factor))
    new_h = max(1, int(h * factor))
    cv2.imshow(window_name, cv2.resize(image, (new_w, new_h)))
    cv2.waitKey(0)
    cv2.destroyAllWindows()

# ---------------------
DEBUG = True

# Load image
image = cv2.imread('../imagesWithTarget/knife1.jpeg')

# Crop to the knife area (adjust as needed)
xBnds = [0, 5000]
yBnds = [500, 1500]
knife = image[yBnds[0]:yBnds[1], xBnds[0]:xBnds[1]]

if DEBUG:
    dispImage(knife, "Original Knife Crop")

# ---------------------
# Downscale for faster processing
scale = 0.2
small_knife = cv2.resize(knife, (0,0), fx=scale, fy=scale)
h_small, w_small = small_knife.shape[:2]

# Initialize mask and models
mask = np.zeros((h_small, w_small), np.uint8)
bgdModel = np.zeros((1,65), np.float64)
fgdModel = np.zeros((1,65), np.float64)

# ---------------------
# Define simple strokes for GrabCut
# Foreground stroke: along the blade
fg_stroke = [int(h_small*0.4), int(h_small*0.6)]
fg_cols = [int(w_small*0.3), int(w_small*0.7)]
mask[fg_stroke[0]:fg_stroke[1], fg_cols[0]:fg_cols[1]] = cv2.GC_FGD

# Background stroke: top and bottom of the image
mask[0:10, :] = cv2.GC_BGD
mask[-10:, :] = cv2.GC_BGD

# ---------------------
# Run GrabCut with mask
cv2.grabCut(small_knife, mask, None, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_MASK)

# Create final mask
mask2 = np.where((mask==cv2.GC_FGD) | (mask==cv2.GC_PR_FGD), 1, 0).astype('uint8')

# Upscale mask to original knife size
mask2 = cv2.resize(mask2, (knife.shape[1], knife.shape[0]), interpolation=cv2.INTER_NEAREST)

# Apply mask
knife_fg = knife * mask2[:, :, np.newaxis]

if DEBUG:
    dispImage(knife_fg, "Foreground Extracted Knife")
