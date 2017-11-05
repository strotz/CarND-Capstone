import rospy
from styx_msgs.msg import TrafficLight
import tensorflow as tf
import cv2
import numpy as np
import os

from camera_monitor import CameraMonitor

class SSDClassifier(object):
    def __init__(self):
        self.graph = self.load_frozen_graph()
        self.sess = tf.Session(graph=self.graph)

        # reference placeholders
        self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.graph.get_tensor_by_name('detection_classes:0')

        self.monitor = CameraMonitor()

        
    def load_frozen_graph(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        frozen_graph_filename = 'learn/nets/simulated_frozen_inference_graph.pb'
        frozen_graph_path = os.path.join(dir_path, frozen_graph_filename)

        graph = tf.Graph()
        with graph.as_default():
            with tf.gfile.GFile(frozen_graph_path, 'rb') as fid:
                serialized_graph = fid.read()
                graph_def = tf.GraphDef()
                graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(graph_def, name='')
        return graph

    def predict(self, image):
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        x = np.asarray(rgb_image, dtype=np.uint8)
        x = np.expand_dims(x, 0)
        (boxes, scores, classes) = self.sess.run( [self.detection_boxes, self.detection_scores, self.detection_classes], 
            feed_dict={self.image_tensor: x})
                                            
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)

        return (boxes, scores, classes)

    def get_classification(self, image):
        (boxes, scores, classes) = self.predict(image)

        if scores.size <= 0:
            return TrafficLight.UNKNOWN
        
        index = np.argmax(scores)
        return classes[index]

