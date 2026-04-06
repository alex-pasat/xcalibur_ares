import roboticstoolbox as rtb
import numpy as np
from math import pi, degrees
import pyautogui
import cv2

data = np.load("knife_data.npz")
q = data["arr_0"]
normals = data["arr_1"]
profile = data["arr_2"]
velocity = data["arr_3"]
start = data["arr_4"]
range_ = data["arr_5"]
mid_start = data["arr_6"]
mid_end = data["arr_7"]

def robot():
    # Define the links
    # First, the underpass
    L1 = rtb.PrismaticMDH(theta=0, a=0, alpha=0, qlim=[0, 198])
    L2 = rtb.RevoluteMDH(a=29, alpha=-pi/2, d=-12.5, qlim=[0, 2*pi])
    L3 = rtb.RevoluteMDH(a=27.5, alpha=pi/2, d=-0, qlim=[0, 2*pi])
    L4 = rtb.RevoluteMDH(a=0, alpha=pi/2, d=68.23, qlim=[0, 2*pi])
    L5 = rtb.PrismaticMDH(theta=0, a=29.74, alpha=0, qlim=[0, 95])

    # Create robot
    robot = rtb.DHRobot([L1, L2, L3, L4, L5], name="Robot")

    return robot

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
        cv2.polylines(output, [contour], isClosed=isClosed, color=(0, 255, 0), thickness=100)
    dispImage(output, text)

dt = 0.1

# initial params
start = q[0]
end = q[-1]

# time step
def time_step(state, idx, start, end, dt, velocity):
    # if at end, reset
    if state[2] >= end:
        state = start
        idx = 0
    # otherwise add the accumulated distance
    else:
        # Check if we advance to next idx
        if state[2] >= velocity[idx+1][2]:
            idx += 1
        # Now we're at right index so we integrate time step with idx velocity
        for i in range(len(state)):
            state[i] += (velocity[idx][i]*dt)
    return state, idx
