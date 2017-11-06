from __future__ import division

from styx_msgs.msg import TrafficLight
from ssd_classifier import SSDClassifier
import cv2
import numpy as np
import rospy

from camera_monitor import CameraMonitor


class HybridClassifier(object):
    def __init__(self):
        self.ssd = SSDClassifier()

        # define range of green, yellow and red colors in HSV for bag
        green_lower = np.array([50,25,150])
        green_upper = np.array([100,255,255])
        self.green = (green_lower, green_upper)

        yellow_lower = np.array([25,25,150])
        yellow_upper = np.array([50,255,255])
        self.yellow = (yellow_lower, yellow_upper)
        
        red_lower = np.array([0,25,150])
        red_upper = np.array([25,255,255])
        self.red = (red_lower, red_upper)

        self.monitor = CameraMonitor()

    def get_classification(self, image):
        (boxes, scores, classes) = self.ssd.predict(image)

        if scores.size <= 0:
            return TrafficLight.UNKNOWN
        
        index = np.argmax(scores)
        nn_score = scores[index]
        if nn_score < 0.01:
            return TrafficLight.UNKNOWN

        nn_class = self.ssd.class_to_tl(classes[index])

        box = boxes[index]
        box_class, box_score = self.process_box(image, box)
        rospy.loginfo("hybrid: %s-%s scores: %s %s", nn_class, box_class, nn_score, box_score)

        if box_class == nn_class:
            return nn_class

        if nn_score > box_score:
            return nn_class
        else:
            return box_class


    def process_box(self, image, box):

        (b, l, t, r) = box
        h, w, c = image.shape

        size = h * w
        if size < 2000:
            return TrafficLight.UNKNOWN, 0.0

        bottom = max(0, int(b*h))
        top = min(h, int(t*h))
        left = max(0, int(l*w))
        right = min(w, int(r*w))

        # rospy.loginfo("%s %s %s %s", bottom, top, left, right)
        roi = image[bottom:top,left:right]
        self.monitor.trace(roi)

        # Convert BGR to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        green_count = self.count_dots(hsv, self.green)
        yellow_count = self.count_dots(hsv, self.yellow)
        red_count = self.count_dots(hsv, self.red)

        # green_mask = self.mask_dots(hsv, self.green)
        # yellow_mask = self.mask_dots(hsv, self.yellow)
        # red_mask = self.mask_dots(hsv, self.red)
        # mask = np.bitwise_or(green_mask, yellow_mask)
        # mask = np.bitwise_or(mask, red_mask)
        # res = cv2.bitwise_and(detected_area, detected_area, mask=mask)
        # self.monitor.trace(res)
        
        norm = green_count + red_count + yellow_count
        if norm == 0:
            return TrafficLight.UNKNOWN, 0

        counts = np.array([red_count, yellow_count, green_count])
        index = np.argmax(counts) 
        return index, counts[index] / norm

    def mask_dots(self, hsv, limit):
        # Threshold the HSV image
        return cv2.inRange(hsv, limit[0], limit[1])

    def count_dots(self, hsv, limit):
        # Threshold the HSV image
        mask = cv2.inRange(hsv, limit[0], limit[1])
        return cv2.countNonZero(mask)

