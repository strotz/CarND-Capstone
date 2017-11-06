import cv2
import numpy as np
import rospy
from styx_msgs.msg import TrafficLight

import contour_classifier
import ssd_classifier
import hybrid_classifier

class TLClassifier(object):
    def __init__(self, method = 'hybrid'):
        if method == 'contour':
            self.classifier = contour_classifier.ContourClassifier()
        elif method == 'ssd':
            self.classifier = ssd_classifier.SSDClassifier()
        elif method == 'hybrid':
            self.classifier = hybrid_classifier.HybridClassifier()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if self.classifier is None:
            rospy.logwarn("classifier was not defined")
            return TrafficLight.UNKNOWN

        return self.classifier.get_classification(image)
