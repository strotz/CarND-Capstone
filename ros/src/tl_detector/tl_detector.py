#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from light_classification.tl_classifier import TLClassifier

import state_filter
from car_camera import CarCamera
from camera_monitor import CameraMonitor

import tf
import cv2
import yaml
import math
import numpy as np
import os
import scipy.misc

from tf.transformations import euler_from_quaternion

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.INFO)

        # data initialization
        self.pose = None
        self.latest_waypoints = None
        self.stop_line_waypoints = None # indexes of waypoints associated with stop lines

        self.lights = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        self.image_pub = rospy.Publisher('/image_tl_select', Image, queue_size=1)

        # config file load
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.camera = CarCamera(self.config['camera_info'])

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state_filter = state_filter.StateFilter()
        self.monitor = CameraMonitor()

        # to collect pictures
        self.dump_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, lane):
        self.latest_waypoints = lane.waypoints
        if (lane.waypoints):
            self.stop_line_waypoints = self.load_stop_line_waypoints(lane.waypoints)
        else:
            self.stop_line_waypoints = None


    def traffic_cb(self, msg):
        self.lights = msg.lights # available only in simulator, array of traffic lights with pose and state

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        light_wp, state = self.process_traffic_lights(msg)

        # to_send = self.state_filter.append(state, light_wp)
        to_send = self.state_filter.bypass(state, light_wp)
        if to_send: 
            self.upcoming_red_light_pub.publish(Int32(to_send))


    def get_closest_waypoint(self, pose, waypoints):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
            waypoins: array of waypoints to search. 

        Returns:
            int: index of the closest waypoint in waypoints

        """
        closest = -1
        closest_distance2 = 1000000000
        ego_x = pose.position.x
        ego_y = pose.position.y

        for (i, wp) in enumerate(waypoints):
            p = wp.pose.pose.position
            heading_x = p.x - ego_x
            heading_y = p.y - ego_y
            distance2 = (heading_x) * (heading_x) + (heading_y) * (heading_y)
            if distance2 < closest_distance2:
                closest = i
                closest_distance2 = distance2

        return closest

    # TODO: code duplication in updater, need library
    def find_next_waypoint(self, pose, waypoints):
        closest = -1
        closest_distance2 = 1000000000
        ego_x = pose.position.x
        ego_y = pose.position.y

        roll, pitch, ego_yaw = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        ego_heading_x = math.cos(ego_yaw)
        ego_heading_y = math.sin(ego_yaw)

        for (i, wp) in enumerate(waypoints):
            p = wp.pose.pose.position
            heading_x = p.x - ego_x
            heading_y = p.y - ego_y
            distance2 = (heading_x) * (heading_x) + (heading_y) * (heading_y)
            if distance2 < closest_distance2:
                correlation = heading_x * ego_heading_x + heading_y * ego_heading_y
                if correlation > 0:
                    closest = i
                    closest_distance2 = distance2

        return closest

    def get_light_state(self, camera_image, pose, light):
        """Determines the current color of the traffic light

        Args:
            pose (Pose): position and orientation of the vihicle
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        coords = self.camera.to_image_coordinates(pose.position, pose.orientation, light.pose.pose.position)
        if coords is None:
            return TrafficLight.UNKNOWN

        if not self.camera.does_it_fit(coords):
            return TrafficLight.UNKNOWN

        (pos_x, pos_y) = coords            
        cv_image = self.bridge.imgmsg_to_cv2(camera_image, "bgr8")
        roi = self.camera.extract_roi(cv_image, pos_x, pos_y)
        self.monitor.trace(roi)

        # Get classification
        state = self.light_classifier.get_classification(roi)

        dump_unknown_images = False
        if state == TrafficLight.UNKNOWN and dump_unknown_images:
            t = rospy.Time.now()
            filename = 'image%s.png' % (t) 
            cv2.imwrite(filename, cv_image)
            filename = 'image%s-roi.png' % (t) 
            cv2.imwrite(filename, roi)
            rospy.loginfo("image %s ROI at %s, %s", t, pos_x, pos_y)

        rospy.logdebug("state: %s", state)
        return state

    def load_stop_line_waypoints(self, waypoints):

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions'] 

        stop_line_waypoints = []
        for lx, ly in stop_line_positions:
            line_pose = Pose()
            line_pose.position.x = lx
            line_pose.position.y = ly
            line_position = self.get_closest_waypoint(line_pose, waypoints) # TODO: o(x^2) implementation, can be better
            stop_line_waypoints.append(line_position)

        return stop_line_waypoints


    def process_traffic_lights(self, camera_image):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        light_wp = -1

        # to prevent issues with multitreaded update
        pose = self.pose
        waypoints = self.latest_waypoints
        stop_line_waypoints = self.stop_line_waypoints
        lights = self.lights

        if pose is None or waypoints is None or lights is None or stop_line_waypoints is None:
            return -1, TrafficLight.UNKNOWN

        car_position_wp = self.find_next_waypoint(pose, waypoints) # index of waypoint in front of the car, it is important to use orientation
        if car_position_wp == -1:
            rospy.logdebug('no waypoints ahead')
            return -1, TrafficLight.UNKNOWN

        # Find the nearest stop line in front of the vehicle.
        stop_line_idx = np.searchsorted(stop_line_waypoints, [car_position_wp,], side='right')[0]
        if stop_line_idx == len(stop_line_waypoints):
            stop_line_idx = 0
        light_wp = stop_line_waypoints[stop_line_idx] # index of waypoint associated with stop line, first part of the result
        
        distance_to_stopline = self.distance(waypoints, car_position_wp, light_wp)
        MAX_DIST = 70  # meters
        if distance_to_stopline > MAX_DIST:
            rospy.logdebug("light is too far: %s meters", distance_to_stopline)
            return -1, TrafficLight.UNKNOWN

        # Find the closest visible traffic light (if one exists)
        stop_line_waypoint = waypoints[light_wp] 
        light_index = self.get_closest_waypoint(stop_line_waypoint.pose.pose, lights)
        if light_index == -1:
            rospy.logdebug('light is not found')
            return -1, TrafficLight.UNKNOWN
        light = lights[light_index]

        use_simulated_red = False
        if use_simulated_red: # this branch will work with simulator
            state = light.state
            return light_wp, state

        state = self.get_light_state(camera_image, pose, light)
        # rospy.logdebug("light is near: %s meters, state: %s", distance_to_stopline, state)
        return light_wp, state


    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        total = len(waypoints)
        ahead = self.how_many_ahead(wp1, wp2, total)
        for i in range(0, ahead+1):
            start = (wp1+i) % total
            next = (start+1) % total  
            dist += dl(waypoints[start].pose.pose.position, waypoints[next].pose.pose.position)

        return dist

    def how_many_ahead(self, current, end, total):
        return (end - current) if (end >= current) else (end + total - current) # to cover circular nature of waypoints


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
