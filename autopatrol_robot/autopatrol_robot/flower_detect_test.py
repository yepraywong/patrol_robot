from cv_bridge import CvBridge  # Convert image format
import cv2  # save image
import numpy as np

cv_bridge_ = CvBridge()
cv_image = cv2.imread(r'/home/rayray/navstudy/nav_ws/img_-4.36_-1.46.png')
# cv_image = cv_bridge_.imgmsg_to_cv2(image)
# Convert to HSV for better color segmentation
if cv_image is not None:
    print('yes')
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    cv2.imshow('Image', hsv_image)
    cv2.waitKey(0)  # 等待按键后关闭窗口
    cv2.destroyAllWindows()
    cv2.imwrite('firehhhh.png', hsv_image)
    # Define the range for flower color in HSV space (e.g., red flowers)
    lower_color = np.array([150, 65, 55])  # lower range of red
    upper_color = np.array([205, 255, 255])  # upper range of red
    # Create a mask that identifies the flower area based on color
    mask = cv2.inRange(cv_image, lower_color, upper_color)
    cv2.imshow('Image', mask)
    cv2.waitKey(0)  # 等待按键后关闭窗口
    cv2.destroyAllWindows()
    # Apply mask to the original image
    result = cv2.bitwise_and(cv_image, cv_image, mask=mask)
    cv2.imshow('Image', result)
    cv2.waitKey(0)  # 等待按键后关闭窗口
    cv2.destroyAllWindows()
    # Find contours of detected flower
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the largest contour, assuming it's the flower
        largest_contour = max(contours, key=cv2.contourArea)
        # Get the bounding box of the flower
        x, y, w, h = cv2.boundingRect(largest_contour)
        # Return the center position of the flower
        flower_center = (x + w / 2, y + h / 2)
        print(flower_center)