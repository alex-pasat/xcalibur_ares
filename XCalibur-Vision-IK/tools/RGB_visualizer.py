import cv2
import numpy as np
import pyautogui

# Load image
image_path = "../rpiImages/orange.jpg"
img_orig = cv2.imread(image_path)
if img_orig is None:
    print("Error: Image not found")
    exit()

# -----------------------------
# Rescale image to fit screen
# -----------------------------
screen_w, screen_h = pyautogui.size()
h, w = img_orig.shape[:2]
scale = min(0.9 * screen_w / w, 0.9 * screen_h / h)
new_w = max(1, int(w * scale))
new_h = max(1, int(h * scale))
img = cv2.resize(img_orig, (new_w, new_h))

# -----------------------------
# Mouse callback
# -----------------------------
def show_rgb(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:  # or EVENT_LBUTTONDOWN for click
        # Map scaled coordinates back to original
        orig_x = int(x / scale)
        orig_y = int(y / scale)

        # Make sure we stay inside bounds
        orig_x = min(max(orig_x, 0), img_orig.shape[1] - 1)
        orig_y = min(max(orig_y, 0), img_orig.shape[0] - 1)

        # Get RGB value from original image
        rgb_value = img_orig[orig_y, orig_x]  # BGR format
        r, g, b = int(rgb_value[2]), int(rgb_value[1]), int(rgb_value[0])

        # Display
        output = img.copy()
        cv2.circle(output, (x, y), 5, (0, 0, 255), -1)
        text = f"X:{orig_x} Y:{orig_y} R:{r} G:{g} B:{b}"
        cv2.putText(output, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        cv2.imshow("RGB Inspector", output)

# -----------------------------
# Setup window
# -----------------------------
cv2.namedWindow("RGB Inspector")
cv2.setMouseCallback("RGB Inspector", show_rgb)
cv2.imshow("RGB Inspector", img)
cv2.waitKey(0)
cv2.destroyAllWindows()