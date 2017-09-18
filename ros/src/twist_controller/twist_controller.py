import math
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.vehicle_mass = args[0]
        self.fuel_capacity = args[1]
        self.brake_deadband = args[2]
        self.decel_limit = args[3]
        self.accel_limit = args[4]
        self.wheel_radius = args[5]
        self.wheel_base = args[6]
        self.steer_ratio = args[7]
        self.max_lat_accel = args[8]
        self.max_steer_angle = args[9]

        self.pid_throttle = PID(0.6, 0.05, 0.1, self.decel_limit, self.accel_limit)
        self.pid_steering = PID(6.0, 0.3, 1.0, -self.max_steer_angle, self.max_steer_angle)
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 0.0, self.max_lat_accel, self.max_steer_angle)

    def control(self, *args, **kwargs):
        proposed_linear_velocity = args[0]
        proposed_angular_velocity = args[1]
        current_linear_velocity = args[2]
        dt = 0.02

        velocity_error = proposed_linear_velocity - current_linear_velocity
        throttle = self.pid_throttle.step(velocity_error, dt)

        current_steer = self.yaw_controller.get_steering(proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity)
        steer_error = proposed_angular_velocity - current_steer
        steer = self.pid_steering.step(steer_error, dt)

        throttle = max(throttle, 0.0)
        brake = math.fabs(min(0.0, throttle))

        return throttle, brake, steer