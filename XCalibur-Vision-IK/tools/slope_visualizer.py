import cv2
import numpy as np
import pyautogui
from collections import defaultdict


# ----------------------------
# Blade contour extraction
# ----------------------------
def getTopContour(hsv):
    # get dimensions
    h, w = hsv.shape[:2]

    # Mask to only take blade coloured pixels
    grey_lwr_thr = np.array([0, 0, 100])
    grey_upr_thr = np.array([180, 50, 255])
    mask = cv2.inRange(hsv, grey_lwr_thr, grey_upr_thr)

    # Denoise mask by morphological closing and opening
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # Extract largest contour of mask to enclose the knife and optionally display
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    blade_contour = max(contours, key=cv2.contourArea)

    # Take upper half of contour
    top_contour = []

    for x in range(w):
        slice = np.where(blade_contour[:, 0, 0] == x)[0]
        if slice.size > 0:
            top_pt = slice[np.argmin(blade_contour[slice, 0, 1])]
            top_contour.append(blade_contour[top_pt:top_pt+1, 0:1, :])  # shape (1,1,2)

    top_contour = np.concatenate(top_contour, axis=0).astype(np.int32) # shape (N,1,2)
    top_contour = top_contour[np.argsort(top_contour[:, 0, 0])] # sort by ascending x values

    return top_contour

def fwd_avg(vals, n):
    if n <= 0:
        raise ValueError("n must be positive")
    if n > len(vals):
        return np.array([])

    kernel = np.ones(n) / n
    return np.convolve(vals, kernel, mode='valid')


# ----------------------------
# Interactive slope viewer
# ----------------------------
def interactiveSlopeViewer(image_path):

    img = cv2.imread(image_path)
    if img is None:
        print("Failed to load image.")
        return

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    contour = getTopContour(hsv)

    if contour is None:
        print("No contour found.")
        return

    # Compute slopes in original resolution
    x = contour[:, 0, 0].astype(np.float32)
    y = contour[:, 0, 1].astype(np.float32)

    dx = np.diff(x)
    dy = np.diff(y)
    slopes = dy / (dx + 1e-6)
    slopes = fwd_avg(slopes,50) # OK technically I am not using slopes anymore

    # ----------------------------
    # Scale image to fit screen
    # ----------------------------
    screen_w, screen_h = pyautogui.size()
    h, w = img.shape[:2]

    scale = min(0.9 * screen_w / w, 0.9 * screen_h / h)
    new_w = int(w * scale)
    new_h = int(h * scale)

    img_small = cv2.resize(img, (new_w, new_h))

    # Scale contour coordinates
    x_scaled = (x * scale).astype(int)
    y_scaled = (y * scale).astype(int)

    contour_scaled = np.stack([x_scaled, y_scaled], axis=1).reshape(-1,1,2)

    # -------------------------------------------------
    # Combine slopes per displayed X column only
    # -------------------------------------------------
    slope_by_x = defaultdict(lambda: -np.inf)
    y_by_x = {}

    for i in range(len(slopes)):
        px = x_scaled[i]
        py = y_scaled[i]

        # Store max abs slope for that x column
        if abs(slopes[i]) > slope_by_x[px]:
            slope_by_x[px] = abs(slopes[i])
            y_by_x[px] = py   # store representative y for drawing

    display = img_small.copy()
    cv2.namedWindow("Contour Viewer")

    def mouse_callback(event, mx, my, flags, param):
        nonlocal display

        if event == cv2.EVENT_MOUSEMOVE:

            display = img_small.copy()
            cv2.polylines(display, [contour_scaled], False, (0,255,0), 2)

            if mx in slope_by_x:

                slope_val = slope_by_x[mx]
                py = y_by_x[mx]

                # Draw representative contour point
                cv2.circle(display, (mx, py), 6, (0,0,255), -1)

                text = f"x={mx}, max|slope|={slope_val:.3f}"
                cv2.putText(display, text, (20,40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0,0,255), 2)

            cv2.imshow("Contour Viewer", display)

    cv2.setMouseCallback("Contour Viewer", mouse_callback)

    cv2.polylines(display, [contour_scaled], False, (0,255,0), 2)
    cv2.imshow("Contour Viewer", display)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# ----------------------------
# CHANGE THIS PATH
# ----------------------------
interactiveSlopeViewer(
    r"C:\Users\h2par\OneDrive\Desktop\School\Terms\4B\MTE 482\Vision\KnifeCrops\knife3\knife30_5.jpeg"
)

