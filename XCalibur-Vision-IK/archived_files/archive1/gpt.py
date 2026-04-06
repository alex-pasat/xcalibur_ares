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
image_path = "../imagesWithTarget\knife1Clutter.jpeg"
img = cv2.imread(image_path)
original = img.copy()

# ---- Convert to grayscale ----
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
dispImage(gray)

# ---- Blur to reduce noise ----
blur = cv2.GaussianBlur(gray, (5,5), 0)
dispImage(blur)

# ---- Edge detection ----
edges = cv2.Canny(blur, threshold1=50, threshold2=100)
dispImage(edges)

# ---- Morphological closing (helps connect blade edges) ----
kernel = np.ones((5,5), np.uint8)
edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
dispImage(edges)

# # ---- Find contours ----
# contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

# # ---- Filter contours by size and horizontal position ----
# blade_contour = None
# max_area = 0

# for cnt in contours:
#     area = cv2.contourArea(cnt)
#     x, y, w, h = cv2.boundingRect(cnt)

#     # Heuristic: blade is wide and relatively thin
#     if w > 300 and h < 200:
#         if area > max_area:
#             max_area = area
#             blade_contour = cnt

# if blade_contour is None:
#     print("Blade contour not found!")
#     exit()

# # ---- Create overlay ----
# overlay = original.copy()

# # Draw contour in red
# cv2.drawContours(overlay, [blade_contour], -1, (0,0,255), 3)

# # ---- Extract profile mask ----
# mask = np.zeros_like(gray)
# cv2.drawContours(mask, [blade_contour], -1, 255, thickness=cv2.FILLED)

# # ---- Show results ----
# plt.figure(figsize=(14,6))

# plt.subplot(1,2,1)
# plt.title("Detected Blade Profile Overlay")
# plt.imshow(cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))
# plt.axis("off")

# plt.subplot(1,2,2)
# plt.title("Extracted Blade Profile Mask")
# plt.imshow(mask, cmap='gray')
# plt.axis("off")

# plt.tight_layout()
# plt.show()
