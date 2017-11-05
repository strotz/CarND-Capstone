import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraMonitor(object):
    def __init__(self, enabled = True, channel_name = '/image_tl_select'):
        self.enabled = enabled
        if enabled:
            self.bridge = CvBridge()
            self.image_pub = rospy.Publisher(channel_name, Image, queue_size=1)

    def trace(self, image):
        if self.enabled:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

    def draw_boxes(self, image, boxes, scores):
        if not self.enabled:
            return

        best = np.argmax(scores)

        h, w, c = image.shape
        for i, box in enumerate(boxes):
            if scores[i] < 0.1 and i != best:
                continue

            bottom, left, top, right = box
            bottom = int(bottom * h)
            top = int(top * h)
            left = int(left * w)
            right = int(right * w)

            cv2.rectangle(image, (left, top), (right, bottom), color=(0,0,255), thickness=3)

        self.trace(image)