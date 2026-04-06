import cv2
import numpy as np
import pyautogui

def dispImage(image):
    screen_width, screen_height = pyautogui.size()
    h, w = image.shape[:2]
    factor = min(0.9*screen_width / w, 0.9*screen_height / h)
    new_w = max(1, int(w * factor))
    new_h = max(1, int(h * factor))
    cv2.imshow("", cv2.resize(image, (new_w, new_h)))
    cv2.waitKey(0)

# Load image
image_path = "../imagesWithTarget/knife1.jpeg"
img = cv2.imread(image_path)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# crop to only include blade
xBnds = [0,5000]
yBnds = [500,1500]
w = xBnds[1]-xBnds[0]
h = yBnds[1]-yBnds[0]
hsv = hsv[yBnds[0]:yBnds[1], xBnds[0]:xBnds[1]]
img = img[yBnds[0]:yBnds[1], xBnds[0]:xBnds[1]]

# Lower and upper HSV bounds for blade color (tweak these)
lower_blade = np.array([0, 0, 100])   # light gray / silver
upper_blade = np.array([180, 50, 255])

mask = cv2.inRange(hsv, lower_blade, upper_blade)

kernel = np.ones((5,5), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

# Assume largest contour is blade
blade_contour = max(contours, key=cv2.contourArea)

# Draw contour
output = img.copy()
cv2.drawContours(output, [blade_contour], -1, (0, 255, 0), 2)

top_contour = []

for x in range(w):
    idx = np.where(blade_contour[:, 0, 0] == x)[0]
    if idx.size > 0:
        best = idx[np.argmin(blade_contour[idx, 0, 1])]
        # Append as a (1,2) array so final shape will be (N,1,2)
        top_contour.append(blade_contour[best:best+1, 0:1, :])  # shape (1,1,2)

# Concatenate all points into a single array of shape (N,1,2)
top_contour = np.concatenate(top_contour, axis=0).astype(np.int32)

# Draw
output = img.copy()
# cv2.drawContours(output, [top_contour], -1, (0, 255, 0), 2)
cv2.polylines(output, [top_contour], isClosed=False, color=(0, 255, 0), thickness=10)

# dispImage(output)

# Compute discrete derivative (dy/dx)
# dy = np.diff(top_contour[:, 0, 1])
# dy_smooth = cv2.GaussianBlur(dy.reshape(-1, 1), (20, 1), 0).flatten()
# threshold = np.mean(np.abs(dy_smooth)) + 5 * np.std(np.abs(dy_smooth))
# change_indices = np.where(np.abs(dy_smooth) > threshold)[0]

# if len(change_indices) > 0:
#     hilt_index = change_indices[0]
#     print("hilt index = " + str(hilt_index))

# Parameters
n = 1  # take every nth point to reduce noise
slope_threshold = 1.0  # adjust depending on your image scale
clampRight = 3800  # example x-coordinate; ignore points left of this

# Extract x and y from top_contour
x_full = top_contour[:, 0, 0].astype(np.float32)
y_full = top_contour[:, 0, 1].astype(np.float32)

# Only keep points with x >= clampRight
mask = x_full >= clampRight
x_filtered = x_full[mask][::n]  # downsample
y_filtered = y_full[mask][::n]

print(x_filtered)

# Compute slopes between consecutive points
dx = np.diff(x_filtered)
dy = np.diff(y_filtered)
slopes = dy / (dx + 1e-6)

print(slopes)

# Find first slope below threshold
hilt_candidate_indices = np.where(slopes > slope_threshold)[0]
# print(hilt_candidate_indices)

# Visualization
# output = img.copy()

# Draw downsampled points used for slope detection
# for i in hilt_candidate_indices:
#     cv2.circle(output, (int(x_filtered[i]),int(y_filtered[i])), 8, (0, 255, 0), -1)

hiltX = x_filtered[hilt_candidate_indices[0]]
hiltMask = top_contour[:,0,0] <= hiltX
final_profile = top_contour[hiltMask]
final_profile = final_profile.astype(np.int32)

output = img.copy()
cv2.circle(output, (int(x_filtered[hilt_candidate_indices[0]]),int(y_filtered[hilt_candidate_indices[0]])), 8, (0, 255, 0), -1)
cv2.polylines(output, [final_profile], isClosed=False, color=(0, 255, 0), thickness=10)

dispImage(output)


