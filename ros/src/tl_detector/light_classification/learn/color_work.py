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
image_path = path.join(image_dir, 'bag_red.png')
cv_image = cv2.imread(image_path)
# green roi = cv_image[35:125, 405:435]
# yellow_roi = cv_image[65:155, 335:365]
red_roi = cv_image[55:145, 115:145]
roi = red_roi

# Convert BGR to HSV
hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

sample = np.uint8([[[255,222,107]]])
hsv_c = cv2.cvtColor(sample,cv2.COLOR_RGB2HSV)
print(hsv_c)

# define range of green color in HSV
green_lower = np.array([50,25,150])
green_upper = np.array([100,255,255])
yellow_lower = np.array([25,25,150])
yellow_upper = np.array([50,255,255])
red_lower = np.array([0,25,150])
red_upper = np.array([25,255,255])

# Threshold the HSV image to get only blue colors
mask = cv2.inRange(hsv, red_lower, red_upper)

# Bitwise-AND mask and original image
res = cv2.bitwise_and(roi, roi, mask=mask)

im_debug(res)

