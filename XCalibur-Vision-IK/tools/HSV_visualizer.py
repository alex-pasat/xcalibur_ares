import cv2
import numpy as np

# Load image
image_path = "../rpiImages/topLight.jpg"
img = cv2.imread(image_path)
import pyautogui

if img is None:
    print("Error: Image not found")
    exit()

# Convert to HSV
hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
screen_width, screen_height = pyautogui.size()
h, w = hsv_img.shape[:2]
factor = min(0.9*screen_width / w, 0.9*screen_height / h)
print(factor)
new_w = max(1, int(w * factor))
new_h = max(1, int(h * factor))
hsv_img = cv2.resize(hsv_img, (new_w, new_h))
img = cv2.resize(img, (new_w, new_h))

# Mouse callback function
def show_hsv(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:  # Use EVENT_LBUTTONDOWN if you want click instead
        hsv_value = hsv_img[y, x]
        output = img.copy()
        # Draw a small circle at the cursor
        cv2.circle(output, (x, y), 5, (0, 0, 255), -1)
        # Put text: coordinates + HSV
        text = f"X:{x} Y:{y} H:{hsv_value[0]} S:{hsv_value[1]} V:{hsv_value[2]}"
        cv2.putText(output, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.imshow("HSV Inspector", output)

# Create window and set mouse callback
cv2.namedWindow("HSV Inspector")
cv2.setMouseCallback("HSV Inspector", show_hsv)

# Show the original image
cv2.imshow("HSV Inspector", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
