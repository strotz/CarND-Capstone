#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node', log_level=rospy.DEBUG)

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        self.controller = Controller(vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle)

        self.twist_cmd = None
        self.dbw_enabled = None
        self.current_velocity = None

        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cmd_cb)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        self.loop()

    def twist_cmd_cb(self, msg):
        self.twist_cmd = msg

    def dbw_enabled_cb(self, msg):
        self.dbw_enabled = msg.data

    def current_velocity_cb(self, msg):
        self.current_velocity = msg

    def loop(self):
        rate = rospy.Rate(50) # 50Hz

        while not rospy.is_shutdown():
            # to avoid sync issues (some fields can be updated by callbacks) work with local variables
            twist_cmd = self.twist_cmd
            current_velocity = self.current_velocity
            dbw_enabled = self.dbw_enabled

            if (twist_cmd is not None) and (current_velocity is not None):
                if dbw_enabled is None:
                    rospy.loginfo('dbw_enabled is not initialized')
                elif dbw_enabled:
                    proposed_linear_velocity = twist_cmd.twist.linear.x
                    proposed_angular_velocity = twist_cmd.twist.angular.z
                    current_linear_velocity = current_velocity.twist.linear.x

                    throttle, brake, steer = self.controller.control(proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity)
                    rospy.logdebug("calculated throttle: %s, brake: %s, steer: %s", throttle, brake, steer)

                    self.publish(throttle, brake, steer)
                else:
                    self.controller.reset()

            rate.sleep()

    # Note that throttle values passed to publish should be in the range 0 to 1. 
    # Brake values passed to publish should be in units of torque (N*m). 
    # The correct values for brake can be computed using the desired acceleration, weight of the vehicle, and wheel radius.
    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        # do not send brake command unless required
        if brake > 0.001: 
            bcmd = BrakeCmd()
            bcmd.enable = True
            bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
            bcmd.pedal_cmd = brake
            self.brake_pub.publish(bcmd)

if __name__ == '__main__':
    DBWNode()
