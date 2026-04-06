import cv2
import numpy as np
import pyautogui
import os

def dispImage(image, text):
    screen_width, screen_height = pyautogui.size()
    h, w = image.shape[:2]
    factor = min(0.9*screen_width / w, 0.9*screen_height / h)
    new_w = max(1, int(w * factor))
    new_h = max(1, int(h * factor))
    cv2.imshow(text, cv2.resize(image, (new_w, new_h)))
    cv2.waitKey(0)

# img = cv2.imread("../homographyDemos/chess/chess.jpg")
# gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# ret, corners = cv2.findChessboardCornersSB(gray, (10,6))

# if ret:
#     print("Found!")
#     img_draw = cv2.drawChessboardCorners(img.copy(), (10,6), corners, ret)
#     cv2.imshow("Corners", img_draw)
#     cv2.waitKey(0)
# else:
#     print("No Checkerboard Found")

def getFilenames(root_dir):
    relative_paths = []

    for _, _, filenames in os.walk(root_dir):
        for filename in filenames:
            relative_paths.append(filename)

    return relative_paths

def homographize(path):
    base = cv2.imread("../homographyDemos/chess/chess.jpg")
    img = cv2.imread("../homographyDemos/chess/" + path)

    grayBase = cv2.cvtColor(base, cv2.COLOR_BGR2GRAY)
    grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCornersSB(grayImg, (10,6))
    retBase, cornersBase = cv2.findChessboardCornersSB(grayBase, (10,6))

    if not ret or not retBase:
        print(f"No chessboard found in {path}")
        return

    # Compute homography
    H, mask = cv2.findHomography(corners, cornersBase, cv2.RANSAC)

    # Map source corners to base coordinates
    corners_mapped = cv2.perspectiveTransform(corners.astype(np.float32), H)
    mapped_center = np.mean(corners_mapped, axis=0)[0]

    # Chessboard bounding box
    x_min, y_min = np.min(corners_mapped[:,0,:], axis=0)
    x_max, y_max = np.max(corners_mapped[:,0,:], axis=0)
    chess_w = x_max - x_min
    chess_h = y_max - y_min

    # Zoom out: fit chessboard in 10x larger canvas
    scale = 4 # 1/10th the size
    canvas_w = int(chess_w * scale)
    canvas_h = int(chess_h * scale)
    canvas_center = np.array([canvas_w / 2, canvas_h / 2])

    # Translation to move scaled center to canvas center
    translation = canvas_center - mapped_center / scale

    # Scaling + translation homography
    S = np.array([
        [1/scale, 0, 0],
        [0, 1/scale, 0],
        [0, 0, 1]
    ])
    T = np.array([
        [1, 0, translation[0]],
        [0, 1, translation[1]],
        [0, 0, 1]
    ])
    H_final = T @ S @ H

    warped = cv2.warpPerspective(img, H_final, (canvas_w, canvas_h))
    dispImage(warped, path)


images = getFilenames(r"C:\Users\h2par\OneDrive\Desktop\School\Terms\4B\MTE 482\Vision\homographyDemos\chess")

for file in images:
    homographize(file)
