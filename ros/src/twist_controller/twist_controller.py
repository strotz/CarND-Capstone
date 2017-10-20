import math
from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.vehicle_mass = args[0]
        self.fuel_capacity = args[1]
        self.brake_deadband = args[2]
        self.decel_limit = args[3]
        self.acceleration_limit = args[4]
        self.wheel_radius = args[5]
        self.wheel_base = args[6]
        self.steer_ratio = args[7]
        self.max_lat_acceleration = args[8]
        self.max_steer_angle = args[9]

        self.pid_throttle = PID(6.0, 0.25, 1.0, self.decel_limit, self.acceleration_limit)
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 0.0, self.max_lat_acceleration, self.max_steer_angle)

        self.acceleration_lpf = LowPassFilter(3, 1)

    def control(self, *args, **kwargs):
        proposed_linear_velocity = args[0]
        proposed_angular_velocity = args[1]
        current_linear_velocity = args[2]
        dt = 0.02

        velocity_error = proposed_linear_velocity - current_linear_velocity
        acceleration = self.pid_throttle.step(velocity_error, dt)
        acceleration = self.acceleration_lpf.filt(acceleration)

        steer = self.yaw_controller.get_steering(proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity)

        if acceleration > 0.0:
            throttle = acceleration
            brake = 0.0
        else:
            throttle = 0.0
            decel = -acceleration
            if decel < self.brake_deadband:
                decel = 0.0

            mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY
            brake = decel * mass * self.wheel_radius

        return throttle, brake, steer

    def reset(self):
        self.pid_throttle.reset()