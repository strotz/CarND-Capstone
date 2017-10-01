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
MAX_SPEED = 20 * 0.447

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.latest_waypoints = None

        self.latest_pose = None
        self.current_velocity = 0 # TODO: not a best way 
        
        self.redlight_wp = -1

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

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x


    def waypoints_cb(self, lane):
        rospy.logdebug("total base waypoints %s", len(lane.waypoints))
        self.latest_waypoints = lane.waypoints        

    def traffic_cb(self, msg):
        self.redlight_wp = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    # NOTE: do not use fields (self.) in this function, pass all data that can be changed by callbacks as a function parameters
    def prepare_and_publish(self, pose, waypoints):
        closest_wp = self.find_next_waypoint(pose, waypoints)
        if closest_wp == -1:
            rospy.logwarn('no waypoints found ahead of car')
            return
  
        # rospy.logdebug("closest next waypoint is %s", closest_wp)

        tail = len(waypoints) - closest_wp
        send = waypoints[ closest_wp : closest_wp + min(LOOKAHEAD_WPS, tail) ]
        if tail < LOOKAHEAD_WPS:
            send.append(waypoints[ 0 : (LOOKAHEAD_WPS - tail)])

        # Slow down if there is a red light in front
        redlight_wp = self.redlight_wp
        current_velocity = self.current_velocity

        if redlight_wp >= 0: # redlight detected ahead
            distance_to_red_light = self.distance(waypoints, closest_wp, redlight_wp) # in meters
            # rospy.logdebug("current speed %s light %s stop %s", current_velocity, distance_to_red_light, distance_to_stop)

            # TODO: it is better to use current velocity and deccelatarion to calculate smoother profile
            SLOW_DISTANCE = 15
            STOP_DISTANCE = 1 
            if distance_to_red_light <= STOP_DISTANCE: 
                rospy.logdebug("STOP from %s", current_velocity)
                send = self.build_stop_profile(send)
            elif distance_to_red_light < SLOW_DISTANCE:
                rospy.logdebug("SLOW from %s", current_velocity)                
                target_wp = self.how_many_ahead(closest_wp, redlight_wp, len(waypoints))
                send = self.build_slowdown_profile(send, current_velocity, min(target_wp,LOOKAHEAD_WPS))    
            else:
                rospy.logdebug("KEEP (*)")
                send = self.build_keepspeed_profile(send, MAX_SPEED)                

        else:
            rospy.logdebug("KEEP")
            send = self.build_keepspeed_profile(send, MAX_SPEED)

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

    # TODO: it need to keep moving car until it reaches target
    def build_slowdown_profile(self, waypoints, current_velocity, target_wp):
        dv = -current_velocity / target_wp if target_wp > 0.005 else 0
        for i in range(len(waypoints)):
            if (i < target_wp):
                v = current_velocity + dv * i
            else:
                v = 0.0
            self.set_waypoint_velocity(waypoints, i, 0.0)
        return waypoints

    def build_keepspeed_profile(self, waypoints, target_velocity):
        for i in range(len(waypoints)):
            self.set_waypoint_velocity(waypoints, i, target_velocity)
        return waypoints

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
