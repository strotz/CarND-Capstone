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

        self.pid_throttle = PID(1.5, 0.25, 0.1, self.decel_limit, self.accel_limit)
        self.yaw_controller = YawController(self.wheel_base, self.steer_ratio, 0.0, self.max_lat_accel, self.max_steer_angle)

    def control(self, *args, **kwargs):
        proposed_linear_velocity = args[0]
        proposed_angular_velocity = args[1]
        current_linear_velocity = args[2]
        dt = 0.02

        velocity_error = proposed_linear_velocity - current_linear_velocity
        accel = self.pid_throttle.step(velocity_error, dt)

        steer = self.yaw_controller.get_steering(proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity)

        if accel > 0.0:
            throttle = accel
            brake = 0.0
        else:
            throttle = 0.0
            decel = -accel
            if decel < self.brake_deadband:
                decel = 0.0

            mass = self.vehicle_mass + self.fuel_capacity * GAS_DENSITY
            brake = decel * mass * self.wheel_radius

        return throttle, brake, steer

    def reset(self):
        self.pid_throttle.reset()