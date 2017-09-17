#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

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
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.current_position = None
        self.closest_wp = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.current_position = msg.pose.position

    def waypoints_cb(self, waypoints):
        map_wp = waypoints.waypoints
        # find closest waypoint
        if self.current_position:
            self.closest_wp = self.closest_waypoint(self.current_position.x, self.current_position.y, map_wp[self.closest_wp:])
        # publish data
        lane = Lane()
        lane.header.frame_id = waypoints.header.frame_id
        lane.header.stamp = rospy.get_rostime()
        lane.waypoints = map_wp[self.closest_wp:self.closest_wp+LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance_sqrt(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

    def closest_waypoint(self, x, y, waypoints):
        closest_len = 100000
        closest_wp = 0;

        for (i, wp) in enumerate(waypoints):
            map_x = wp.pose.pose.position.x
            map_y = wp.pose.pose.position.y
            dist = self.distance_sqrt(x, y, map_x, map_y)
            if dist < closest_len:
                closet_len = dist
                closest_wp = i

        return closest_wp


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
