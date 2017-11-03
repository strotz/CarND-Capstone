import cv2
import numpy as np
import rospy
from styx_msgs.msg import TrafficLight

import countor_classifier

class TLClassifier(object):
    def __init__(self, method = 'countor'):
        if method == 'countor':
            self.classifier = countor_classifier.CountorClassifier()

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
