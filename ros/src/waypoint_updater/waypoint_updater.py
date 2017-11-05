#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import copy

from tf.transformations import euler_from_quaternion

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.INFO)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.latest_waypoints = None
        self.latest_pose = None
        self.latest_pose_timestamp = None
        self.stop_line_wp = -1

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            latest_pose = self.latest_pose
            latest_waypoints = self.latest_waypoints
            if (latest_pose is not None) and (latest_waypoints is not None):
                self.prepare_and_publish(latest_pose, latest_waypoints)

            rate.sleep()

    def pose_cb(self, msg):
        self.latest_pose = msg.pose
        self.latest_pose_timestamp = msg.header.stamp

    def waypoints_cb(self, lane):
        rospy.logdebug("total base waypoints %s", len(lane.waypoints))
        self.latest_waypoints = lane.waypoints        

    def traffic_cb(self, msg):
        self.stop_line_wp = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    # NOTE: do not use fields (self.) in this function, pass all data that can be changed by callbacks as a function parameters
    def prepare_and_publish(self, pose, waypoints):
        closest_wp = self.find_next_waypoint(pose, waypoints)
        if closest_wp == -1:
            rospy.logwarn('no waypoints found ahead of car')
            return
  
        tail = len(waypoints) - closest_wp 
        send = waypoints[closest_wp:closest_wp + min(LOOKAHEAD_WPS, tail)]
        if tail < LOOKAHEAD_WPS:
            rospy.logdebug("TAIL: closest next waypoint: %s, total: %s send: %s", closest_wp, len(waypoints), len(send))
            rospy.logdebug("TAIL: need to fill %s", LOOKAHEAD_WPS-tail)
            send.extend(waypoints[0:(LOOKAHEAD_WPS-tail)])

        # ensure that we are not changing original waypoints
        MAX_SPEED=self.get_waypoint_velocity(send[0])
        send = copy.deepcopy(send)

        # Slow down if there is a red light in front
        stop_line_wp = self.stop_line_wp
        current_velocity = self.get_waypoint_velocity(send[0])
 
        if stop_line_wp >= 0: # redlight detected ahead
            distance_to_red_light = self.distance(waypoints, closest_wp, stop_line_wp) # in meters

            MARGIN = 30
            SLOW_DISTANCE = 30
            STOP_DISTANCE = 5 
            if distance_to_red_light <= STOP_DISTANCE: 
                rospy.logdebug("RUN: STOP distance to light %s", distance_to_red_light)
                send = self.build_stop_profile(send)
            elif distance_to_red_light < (SLOW_DISTANCE + MARGIN):
                rospy.logdebug("RUN: SLOW distance to light %s", distance_to_red_light)                
                send = self.build_slowdown_profile(send, distance_to_red_light, SLOW_DISTANCE, STOP_DISTANCE, MAX_SPEED)    
            else:
                rospy.logdebug("RUN: KEEP distance to light %s", distance_to_red_light)
                send = self.build_keepspeed_profile(send, MAX_SPEED)                

        else:
            rospy.logdebug("RUN: KEEP")
            send = self.build_keepspeed_profile(send, MAX_SPEED)
        
        rospy.logdebug("RUN: send[0]=%s", self.get_waypoint_velocity(send[0]))

        # publish data
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = send
        self.final_waypoints_pub.publish(lane)

    def build_stop_profile(self, waypoints):
        for i in range(len(waypoints)):
            self.set_waypoint_velocity(waypoints, i, 0.0)
        return waypoints

    # it keeps moving car until it reaches target
    def build_slowdown_profile(self, send, distance_to_red_light, SLOW_DISTANCE, STOP_DISTANCE, MAX_SPEED):
        way_to_go = distance_to_red_light
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(0, len(send)):
            if way_to_go > SLOW_DISTANCE: # margin
                self.set_waypoint_velocity(send, i, MAX_SPEED) 
            elif way_to_go > STOP_DISTANCE: # slow down
                a = (way_to_go - STOP_DISTANCE) / (SLOW_DISTANCE - STOP_DISTANCE) # between 0 and 1
                v = a * a * MAX_SPEED # x^2
                self.set_waypoint_velocity(send, i, v)
            else: # complete stop
                self.set_waypoint_velocity(send, i, 0.0)

            if i < (len(send) - 1): # we can do it because it prepares way_to_go for next iteration
                dist = dl(send[i].pose.pose.position, send[i + 1].pose.pose.position)
            way_to_go -= dist

        return send

    def build_keepspeed_profile(self, send, target_velocity):
        for i in range(len(send)):
            self.set_waypoint_velocity(send, i, target_velocity)
        return send

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

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

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
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
