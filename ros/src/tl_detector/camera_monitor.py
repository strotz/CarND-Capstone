import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraMonitor(object):
    def __init__(self, enabled = True, channel_name = '/image_tl_select'):
        self.enabled = enabled
        if enabled:
            self.bridge = CvBridge()
            self.image_pub = rospy.Publisher(channel_name, Image, queue_size=1)

    def trace(self, image):
        if self.enabled:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
