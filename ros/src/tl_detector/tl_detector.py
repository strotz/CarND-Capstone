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
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''

        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED or state == TrafficLight.YELLOW else -1
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

    @staticmethod
    def to_car_coordinates(car_position, car_orientation, point_in_world):
        l = point_in_world

        pos = car_position
        ori = car_orientation

        theta = math.atan2(2.0 * (ori.w * ori.z + ori.x * ori.y), 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z))
        dx = l.x - pos.x
        dy = l.y - pos.y

        X = dy * math.sin(theta) + dx * math.cos(theta)
        Y = dy * math.cos(theta) - dx * math.sin(theta)

        return (X, Y)


    def project_to_image_plane_1(self, pose, point_in_world):
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
        camera_position = pose.position
        camera_orientation = pose.orientation

        px = point_in_world.x
        py = point_in_world.y
        pz = point_in_world.z
        
        xt = camera_position.x
        yt = camera_position.y
        zt = camera_position.z

        #Override focal lengths with data from site for testing
        #fx = 1345.200806
        #fy = 1353.838257 

        #Convert rotation vector from quaternion to euler:
        roll, pitch, camera_yaw = euler_from_quaternion([camera_orientation.x, camera_orientation.y, camera_orientation.z, camera_orientation.w])
        sin_yaw = math.sin(camera_yaw)
        cos_yaw = math.cos(camera_yaw)

        #Rotation followed by translation
        Rnt = (px*cos_yaw - py*sin_yaw + xt, px*sin_yaw + py*cos_yaw + yt, pz + zt)   

        #Pinhole camera model w/o distorion
        u = int(fx * -Rnt[1]/Rnt[0] + image_width/2)
        v = int(fy * -Rnt[2]/Rnt[0] + image_height/2)
     
        return (u, v)

    def project_to_image_plane(self, pose, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            pose (Pose): position and orientation of the car
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']
      
        # Use transform and rotation to calculate 2D position of light in image
        X, Y = TLDetector.to_car_coordinates(pose.position, pose.orientation, point_in_world)

        pts = np.array([[X, Y, 0.0]], dtype=np.float32)
        mat = np.array([[fx,  0, image_width / 2],
                        [ 0, fy, image_height / 2],
                        [ 0,  0,  1]], dtype=np.float32)
        proj, d = cv2.projectPoints(pts, (0,0,0), (0,0,0), mat, None)

        x = int(proj[0,0,0])
        y = int(proj[0,0,1])
        return (x, y)


    def get_light_state(self, pose, light):
        """Determines the current color of the traffic light

        Args:
            pose (Pose): position and orientation of the vihicle
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # pr = self.project_to_image_plane(pose, light.pose.pose.position)
        pr = self.project_to_image_plane_1(pose, light.pose.pose.position)
        if pr is None:
            return TrafficLight.UNKNOWN
        x, y = pr

        # use light location to zoom in on traffic light in image
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        lw = int(image_width / 4)
        lh = int(image_height / 3)

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

        if pose is None or waypoints is None or lights is None or stop_line_waypoints is None:
            return -1, TrafficLight.UNKNOWN

        car_position_wp = self.find_next_waypoint(pose, waypoints) # index of waypoint in front of the car, it is important to use orientation
        if car_position_wp == -1:
            rospy.logdebug('no waypoints ahead')
            return -1, TrafficLight.UNKNOWN

        # Find the nearest stop line in front of the vehicle.
        stop_line_idx = np.searchsorted(stop_line_waypoints, [car_position_wp,], side='right')[0]
        light_wp = stop_line_waypoints[stop_line_idx] # index of waypoint associated with stop line, first part of the result
        
        distance_to_stopline = self.distance(waypoints, car_position_wp, light_wp)
        MAX_DIST = 80  # meters
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

        X, Y = TLDetector.to_car_coordinates(pose.position, pose.orientation, light.pose.pose.position)
        if X < 5:
            rospy.logdebug('light is back from the car')
            return -1, TrafficLight.UNKNOWN

        use_simulated_red = False
        if use_simulated_red: # this branch will work with simulator
            state = light.state
            return light_wp, state

        state = self.get_light_state(pose, light)
        rospy.logdebug("light is near: %s meters, state: %s", distance_to_stopline, state)
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
