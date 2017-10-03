import cv2
import numpy as np
import rospy
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        h, w, c = image.shape
        total_area = h * w

        if total_area < 10000:
            rospy.logdebug('Light is too small')
            return TrafficLight.UNKNOWN

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        inv = cv2.bitwise_not(gray)
        _, th = cv2.threshold(inv, 220, 255, cv2.THRESH_TOZERO)
        _, contours, _ = cv2.findContours(th, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        cntr = []
        for c in contours:
            area = cv2.contourArea(c) / total_area
            if 0.01 < area and area < 0.2:
                cntr.append(c)

        mask = np.zeros_like(image[:, :, 0])
        mask = cv2.drawContours(mask, cntr, -1, 255, -1)
        cut = cv2.bitwise_and(image, image, mask=mask)

        blue = cv2.calcHist([cut], [0], None, [3], [100, 250]).item(2)
        green = cv2.calcHist([cut], [1], None, [3], [100, 250]).item(2)
        red = cv2.calcHist([cut], [2], None, [3], [100, 250]).item(2)

        norm = blue + green + red
        if norm > 0:
            b = blue / norm
            g = green / norm
            r = red / norm

            if g > 0.4 and r > 0.4:
                rospy.logdebug('Light is YELLOW')
                return TrafficLight.YELLOW
            elif r > 0.5:
                rospy.logdebug('Light is RED')
                return TrafficLight.RED
            elif g > 0.5:
                rospy.logdebug('Light is GREEN')
                return TrafficLight.GREEN
            else:
                rospy.logdebug('Light is UNKNOWN')
                return TrafficLight.UNKNOWN

        rospy.logwarn("norm is 0")
        return TrafficLight.UNKNOWN

