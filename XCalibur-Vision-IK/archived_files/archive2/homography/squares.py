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

def findCorners(img, debug=False):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
 
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (7,6), None)

    corners = corners.reshape(-1, 2)

    return corners

# def sortCenters(points):
#     pts = np.array(points, dtype=np.float32)

#     if len(pts) != 13:
#         raise ValueError("Expected 13 points!")
    
#     pts_sorted_y = pts[np.argsort(pts[:, 1])]

#     # split by row
#     rows = [[] for i in range(4)]
#     rows[0] = pts_sorted_y[:4]   #4 pts
#     rows[1] = pts_sorted_y[4:8]  #4 pts
#     rows[2] = pts_sorted_y[8:11]  #3 pts
#     rows[3] = pts_sorted_y[11:13] #2 pts

#     # sort rows by x
#     for row in rows:
#         row = row[np.argsort(row[:,0])]

#     # join rows
#     ordered = []
#     for i in range(len(rows)):
#         row = rows[i]
#         for j in range(len(row)):
#             ordered.append(row[j])

#     return np.array(ordered, dtype=np.float32)

# def homographize(path):
#     hsvBase, imgBase = loadHSV("../homographyDemos/base.jpg")
#     hsv,img = loadHSV("../homographyDemos/" + path)
#     hsvBase = cv2.resize(hsvBase,(int(img.shape[1]), int(img.shape[1])//int(imgBase.shape[1])*int(imgBase.shape[0])),interpolation=cv2.INTER_CUBIC)
#     imgBase = cv2.resize(imgBase,(int(img.shape[1]), int(img.shape[1])//int(imgBase.shape[1])*int(imgBase.shape[0])),interpolation=cv2.INTER_CUBIC)
#     centersBase = getCenters(hsvBase, imgBase, debug=False)
#     centers = getCenters(hsv, img, debug=False)
#     sortedCentersBase = sortCenters(centersBase)
#     sortedCenters = sortCenters(centers)
#     for i in range(len(sortedCenters)):
#         output = img.copy()
#         h, w = output.shape[:2]
#         thickness = int(0.005 * h)
#         cv2.circle(output, (int(sortedCenters[i,0]), int(sortedCenters[i,1])), thickness, (0, 255, 0), -1)
#         dispImage(output, "center no. " + str(i))

#     H, mask = cv2.findHomography(sortedCenters, sortedCentersBase, cv2.RANSAC)

#     hBase, wBase = imgBase.shape[:2]
#     warped = cv2.warpPerspective(img, H, (int(2*wBase), int(2*hBase)))
#     dispImage(warped, path)

images = getFilenames(r"C:\Users\h2par\OneDrive\Desktop\School\Terms\4B\MTE 482\Vision\homographyDemos\chess")

for file in images:
    print(file)
    # homographize(file)
    hsv,img = loadHSV("../homographyDemos/chess/" + file)
    dispImage(img, "original")
    corners = findCorners(img, debug=True)

    for (cx, cy) in corners:
        output = img.copy()
        h, w = output.shape[:2]

        thickness = int(0.005 * h)

        # draw small filled circle at center
        cv2.circle(output, (cx, cy), thickness, (0, 255, 0), -1)  # green dot

        dispImage(output, file)
    
    # sorted_centers = sortCenters(centers)

    # for (cx, cy) in sorted_centers:
    #     output = img.copy()
    #     h, w = output.shape[:2]
    #     thickness = int(0.005 * h)

    #     # draw small filled circle at center
    #     cv2.circle(output, (cx, cy), thickness, (0, 255, 0), -1)  # green dot

    #     dispImage(output, file)

# hsv,img = loadHSV("../homographyDemos/squares/squares.jpg")
# centers = getCenters(hsv, img, debug=True)
