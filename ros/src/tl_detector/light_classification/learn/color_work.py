import os
import cv2
import numpy as np

from os import path
current_dir = path.dirname(path.abspath(__file__))

def im_debug(img):
    cv2.namedWindow('dst_rt', cv2.WINDOW_NORMAL)
    cv2.imshow('dst_rt', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

image_dir = path.join(current_dir, '../../../../../imgs')
image_path = path.join(image_dir, 'bag_green.png')
cv_image = cv2.imread(image_path)
# green roi = cv_image[35:125, 405:435]
roi = cv_image

# Convert BGR to HSV
hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

sample = np.uint8([[[255,255,255]]])
hsv_c = cv2.cvtColor(sample,cv2.COLOR_RGB2HSV)
print(hsv_c)

# define range of blue color in HSV
lower = np.array([50,25,150])
upper = np.array([100,255,255])

# Threshold the HSV image to get only blue colors
mask = cv2.inRange(hsv, lower, upper)

# Bitwise-AND mask and original image
res = cv2.bitwise_and(roi, roi, mask=mask)

im_debug(res)

