#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import numpy as np
import os
import scipy.misc

from tf.transformations import euler_from_quaternion


STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector', log_level=rospy.DEBUG)

        self.pose = None
        
        self.latest_waypoints = None
        self.stop_line_waypoints = None # indexes of waypoints associated with stop lines

        self.camera_image = None
        
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

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

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
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''

        # TODO: state is always 4 (unknown), should be 0 (red) sometimes
        rospy.logdebug("state %s self.state %s and count %s", state, self.state, self.state_count)
        if state == TrafficLight.RED:
            rospy.loginfo("RED")

        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

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


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")
            return None

        # Use transform and rotation to calculate 2D position of light in image
        l = point_in_world

        pos = self.pose.position
        ori = self.pose.orientation

        theta = math.atan2(2.0 * (ori.w * ori.z + ori.x * ori.y), 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z))
        dx = l.x - pos.x
        dy = l.y - pos.y

        X = dy * math.sin(theta) + dx * math.cos(theta)
        Y = dy * math.cos(theta) - dx * math.sin(theta)

        pts = np.array([[X, Y, 0.0]], dtype=np.float32)
        mat = np.array([[fx,  0, image_width / 2],
                        [ 0, fy, image_height / 2],
                        [ 0,  0,  1]], dtype=np.float32)
        proj, d = cv2.projectPoints(pts, (0,0,0), (0,0,0), mat, None)

        x = int(proj[0,0,0])
        y = int(proj[0,0,1])
        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        l = light.pose.pose.position
        c = self.pose.position
        dist = math.sqrt((l.x - c.x) ** 2 + (l.y - c.y) ** 2 + (l.z - c.z) ** 2)

        MIN_DIST = 50 # meters
        if dist > MIN_DIST:
            # rospy.logdebug('light is too far: ' + str(dist) + 'm')
            return TrafficLight.UNKNOWN

        pr = self.project_to_image_plane(l)
        if pr is None:
            return TrafficLight.UNKNOWN
        x, y = pr

        # rospy.logdebug('x = ' + str(x) + ' y = ' + str(y))

        # use light location to zoom in on traffic light in image
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        lw = int(image_width / 4)
        lh = int(image_height / 4)

        y = y + lh / 2

        top = int(y - lh)
        bottom = int(y + lh)
        left = int(x - lw)
        right = int(x + lw)

        if top < 0 or bottom > image_height or left < 0 or right > image_width:
            rospy.logdebug('Invalid ROI: ' + str(top) + ', ' + str(bottom) + ', ' + str(left) + ', ' + str(right))
            return TrafficLight.UNKNOWN

        roi = cv_image[top:bottom,left:right]

        # Get classification
        return self.light_classifier.get_classification(roi)

    def load_stop_line_waypoints(self, waypoints): # TODO: seems like we need to do it once on waypoint update

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


    def process_traffic_lights(self):
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

        if (pose and waypoints and stop_line_waypoints):
            car_position_wp = self.find_next_waypoint(pose, waypoints) # index of waypoint in front of the car

            # Find the nearest stop line in front of the vehicle.
            stop_line_idx = np.searchsorted(stop_line_waypoints, [car_position_wp,], side='right')[0]
            light_wp = stop_line_waypoints[stop_line_idx] # index of waypoint associated with stop line, first part of the result

            # Find the closest visible traffic light (if one exists)
            if (lights):
                stop_line_waypoint = waypoints[light_wp] 
                light_index = self.get_closest_waypoint(stop_line_waypoint.pose.pose, lights)
                state = lights[light_index].state

                rospy.loginfo("car %s stop %s light %s state %s", car_position_wp, light_wp, light_index, state)
                return light_wp, state
            # was light = self.lights[stop_line_idx]


        # if light:
        #     state = self.get_light_state(light)
        #     return light_wp, state

        # self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
