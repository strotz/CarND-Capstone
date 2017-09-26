#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

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
MAX_SPEED = 10.0
MIN_SPEED = 0.0

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.INFO)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.latest_waypoints = None
        self.latest_pose = None
        self.latest_wp = None
        self.redlight_wp = None
        self.slow_down = False
        self.velocities = []

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

    def waypoints_cb(self, lane):
        rospy.logdebug("total base waypoints %s", len(lane.waypoints))
        self.latest_waypoints = lane.waypoints
        

    def traffic_cb(self, msg):
        self.redlight_wp = msg.data
        if self.redlight_wp > 0:
            dist = self.distance(self.latest_waypoints, self.latest_wp, self.redlight_wp)
            if dist < 50:
                n_wp = self.redlight_wp - self.latest_wp
                v = max(self.get_waypoint_velocity(self.latest_waypoints[self.latest_wp]), MIN_SPEED)
                dv = v / n_wp
                next_v = v
                self.velocities = []
                for i in range(n_wp):
                    next_v -= dv
                    self.velocities.append(max(next_v, 0.0))
                self.slow_down = True
            else:
                self.slow_down = False
        else:
            self.slow_down = False

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    # NOTE: do not use fields (self.) in this function, pass all data that can be changed by callbacks as a function parameters
    def prepare_and_publish(self, pose, waypoints):
        closest_wp = self.find_next_waypoint(pose, waypoints)
        self.latest_wp = closest_wp
        if closest_wp == -1:
            rospy.logwarn('no waypoints found ahead of car')
            return
  
        rospy.logdebug("next waypoint is %s", closest_wp)

        tail = len(waypoints) - closest_wp
        send = waypoints[ closest_wp : closest_wp + min(LOOKAHEAD_WPS, tail) ]
        if tail < LOOKAHEAD_WPS:
            send.append(waypoints[ 0 : (LOOKAHEAD_WPS - tail)])

        rospy.logdebug("lenght of waypoints to send is %s", len(send))

        # set velocities
        for i in range(LOOKAHEAD_WPS):
            if self.slow_down:
                if (i < len(self.velocities)):
                    v = self.velocities[i]
                else:
                    v = 0.0
            else:
                v = MAX_SPEED
            self.set_waypoint_velocity(send, i, v)

        # publish data
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = send
        self.final_waypoints_pub.publish(lane)
        
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
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
