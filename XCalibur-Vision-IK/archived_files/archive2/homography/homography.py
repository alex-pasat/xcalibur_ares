import cv2
import numpy as np
import pyautogui
import os

def loadHSV(path):
    img = cv2.imread(path)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    return hsv, img

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

def getFilenames(root_dir):
    relative_paths = []

    for _, _, filenames in os.walk(root_dir):
        for filename in filenames:
            relative_paths.append(filename)

    return relative_paths

def getCenters(hsv,img, debug=False):
    h, w = hsv.shape[:2]

    # hsv = cv2.GaussianBlur(hsv, (5, 5), 0)
    # if debug: dispImage(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), "blurred")

    lower_blue = np.array([105, 50, 0])
    upper_blue = np.array([120, 255, 255])

    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    if debug: dispImage(mask_blue, "mask blue")

    kernel = np.ones((15,15), np.uint8)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
    if debug: dispImage(mask_blue, "closed mask blue")
    # kernel = np.ones((7,7), np.uint8)
    # mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
    # dispImage(mask_blue, "opened mask blue")

    contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centers = []

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 1000:  # filter small noise
            continue

        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        centers.append((cx, cy))

        output = img.copy()

    for (cx, cy) in centers:
        thickness = int(0.005 * h)

        # draw small filled circle at center
        cv2.circle(output, (cx, cy), thickness, (0, 255, 0), -1)  # green dot

        # optional: draw crosshair instead
        cv2.line(output, (cx-10, cy), (cx+10, cy), (0,255,0), thickness)
        cv2.line(output, (cx, cy-10), (cx, cy+10), (0,255,0), thickness)

    if debug: dispImage(output, "centers drawn")

    return centers

def sortCenters(points):
    pts = np.array(points, dtype=np.float32)

    if len(pts) != 15:
        raise ValueError("Expected 15 points!")

    pts_sorted_y = pts[np.argsort(pts[:, 1])]

    # 2️⃣ Split into two rows
    top_row = pts_sorted_y[:9]      # 3 triangles × 3 pts
    bottom_row = pts_sorted_y[9:]   # 2 triangles × 3 pts

    # 3️⃣ Sort each row by x (left to right)
    top_row = top_row[np.argsort(top_row[:, 0])]
    bottom_row = bottom_row[np.argsort(bottom_row[:, 0])]

    # 4️⃣ Group into triangles (chunks of 3)
    top_triangles = [top_row[i:i+3] for i in range(0, 9, 3)]
    bottom_triangles = [bottom_row[i:i+3] for i in range(0, 6, 3)]

    # 5️⃣ Sort points inside each triangle for consistency
    # (sort by y, then x to make ordering deterministic)
    for i in range(len(top_triangles)):
        tri = top_triangles[i]
        tri = tri[np.argsort(tri[:, 1])]
        tri[:2] = tri[np.argsort(tri[:2, 0])]
        top_triangles[i] = tri

    for i in range(len(bottom_triangles)):
        tri = bottom_triangles[i]
        tri = tri[np.argsort(tri[:, 1])]
        tri[:2] = tri[np.argsort(tri[:2, 0])]
        bottom_triangles[i] = tri

    # 6️⃣ Flatten in physical layout order
    ordered = []

    # Top row: left → right
    for tri in top_triangles:
        for i in range(len(tri)):
            ordered.append(tri[i])

    # Bottom row: (skip bottom-left) → middle → right
    for tri in bottom_triangles:
        for i in range(len(tri)):
            ordered.append(tri[i])

    return np.array(ordered, dtype=np.float32)

def homographize(path):
    hsvBase, imgBase = loadHSV("../homographyDemos/base.jpg")
    hsv,img = loadHSV("../homographyDemos/" + path)
    hsvBase = cv2.resize(hsvBase,(int(img.shape[1]), int(img.shape[1])//int(imgBase.shape[1])*int(imgBase.shape[0])),interpolation=cv2.INTER_CUBIC)
    imgBase = cv2.resize(imgBase,(int(img.shape[1]), int(img.shape[1])//int(imgBase.shape[1])*int(imgBase.shape[0])),interpolation=cv2.INTER_CUBIC)
    centersBase = getCenters(hsvBase, imgBase, debug=False)
    centers = getCenters(hsv, img, debug=False)
    sortedCentersBase = sortCenters(centersBase)
    sortedCenters = sortCenters(centers)
    # for i in range(len(sortedCenters)):
    #     output = img.copy()
    #     h, w = output.shape[:2]
    #     thickness = int(0.005 * h)
    #     cv2.circle(output, (int(sortedCenters[i,0]), int(sortedCenters[i,1])), thickness, (0, 255, 0), -1)
    #     dispImage(output, "center no. " + str(i))

    H, mask = cv2.findHomography(sortedCenters, sortedCentersBase, cv2.RANSAC)

    hBase, wBase = imgBase.shape[:2]
    warped = cv2.warpPerspective(img, H, (int(2*wBase), int(2*hBase)))
    dispImage(warped, path)

# images = getFilenames(r"C:\Users\h2par\OneDrive\Desktop\School\Terms\4B\MTE 482\Vision\homographyDemos")

# for file in images:
#     homographize(file)
    # hsv,img = loadHSV("../homographyDemos/" + file)
    # centers = getCenters(hsv, debug=False)

    # output = img.copy()
    # h, w = output.shape[:2]
    # for (cx, cy) in centers:
    #     thickness = int(0.005 * h)

    #     # draw small filled circle at center
    #     cv2.circle(output, (cx, cy), thickness, (0, 255, 0), -1)  # green dot

    #     # optional: draw crosshair instead
    #     cv2.line(output, (cx-10, cy), (cx+10, cy), (0,255,0), thickness)
    #     cv2.line(output, (cx, cy-10), (cx, cy+10), (0,255,0), thickness)
    # dispImage(output, file)

