from ssd_classifier import SSDClassifier

class HybridClassifier(SSDClassifier):
    def __init__(self):
        SSDClassifier.__init__()

    def get_classification(self, image):
        (boxes, scores, classes) = super().predict()

        if scores.size <= 0:
            return TrafficLight.UNKNOWN
        
        index = np.argmax(scores)

        box = boxes[index]
        (b, l, t, r) = box
        h, w, c = image.shape

        bottom = int(b*h)
        left = int(l*w)
        top = int(t*h)
        right = int(r*w)

        if bottom < 0:
            bottom = 0
        if top > h:
            top = h
        if left < 0:
            left = 0
        if right > w:
            right = w

        rospy.loginfo("%s %s %s %s", bottom, top, left, right)
        detected_area = image[bottom:top,left:right]

        # TODO: adjust color detection

        self.monitor.trace(detected_area)
        return classes[index]
