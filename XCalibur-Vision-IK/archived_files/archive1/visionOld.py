import cv2
import numpy as np

# --- Load and resize image ---
image = cv2.imread('../imagesOld/knifeCrop.jpg')

# --- Step 1: Rough mask using GrabCut ---
mask = np.zeros(image.shape[:2], np.uint8)
bgdModel = np.zeros((1, 65), np.float64)
fgdModel = np.zeros((1, 65), np.float64)

# Define a rough rectangle around the blade area (adjust these!)
h, w = image.shape[:2]
rect = (int(w*0), int(h*0), int(w*0.95), int(h*0.95))

# Run GrabCut
cv2.grabCut(image, mask, rect, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_RECT)

# Create final mask (1 for sure/possible foreground, 0 for background)
mask2 = np.where((mask==2)|(mask==0), 0, 1).astype('uint8')
foreground = image * mask2[:, :, np.newaxis]

cv2.imshow("Foreground Extracted", foreground)
cv2.waitKey(0)

# --- Step 2: Threshold bright areas (on the extracted foreground) ---
gray = cv2.cvtColor(foreground, cv2.COLOR_BGR2GRAY)
_, blade_mask = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)

# Clean up mask
kernel = np.ones((11, 11), np.uint8)
blade_mask = cv2.morphologyEx(blade_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

# --- Step 3: Edge detection & contouring ---
edged = cv2.Canny(blade_mask, 100, 220, apertureSize=7)
cv2.imshow('Edges', edged)
cv2.waitKey(0)

cv2.destroyAllWindows()

# Get coordinates of all edge points
ys, xs = np.where(edged > 0)

# For each x, find the lowest (max y)
lowest_points = []
for x in np.unique(xs):
    y_vals = ys[xs == x]
    lowest_y = np.max(y_vals)
    lowest_points.append([x, lowest_y])

lowest_points = np.array(lowest_points, dtype=np.int32)

# Draw these points on the image
output = image.copy()
for (x, y) in lowest_points:
    cv2.circle(output, (x, y), 1, (0, 0, 255), -1)

cv2.imshow("Lower Edge Points", output)
cv2.waitKey(0)
cv2.destroyAllWindows()

# Compute discrete derivative (dy/dx)
dy = np.diff(lowest_points[:, 1])

# Optionally smooth it a bit (to ignore small noise)
dy_smooth = cv2.GaussianBlur(dy.reshape(-1, 1), (9, 1), 0).flatten()

# Threshold for sharp vertical change
threshold = np.mean(np.abs(dy_smooth)) + 2 * np.std(np.abs(dy_smooth))

# Indices where large upward movement happens
change_indices = np.where(np.abs(dy_smooth) > threshold)[0]

if len(change_indices) > 0:
    hilt_index = change_indices[0]  # first big change
    hilt_point = lowest_points[hilt_index]
    print("Hilt detected at:", hilt_point)


output = image.copy()
for (x, y) in lowest_points:
    cv2.circle(output, (x, y), 1, (0, 0, 255), -1)
cv2.circle(output, tuple(hilt_point), 8, (0, 255, 0), -1)

cv2.imshow("Hilt Detection", output)
cv2.waitKey(0)
cv2.destroyAllWindows()



blade_pts = lowest_points[:hilt_index]

# Draw these points on the image
output = image.copy()
for (x, y) in blade_pts:
    cv2.circle(output, (x, y), 1, (0, 0, 255), -1)

cv2.imshow("blade pts", output)
cv2.waitKey(0)
cv2.destroyAllWindows()
