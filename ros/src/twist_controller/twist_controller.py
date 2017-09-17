from pid import PID

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

        self.pid_velocity = PID(0.3, 0.0001, 6.0, self.decel_limit, self.accel_limit)

    def control(self, *args, **kwargs):
        proposed_linear_velocity = args[0]
        proposed_angular_velocity = args[1]
        current_linear_velocity = args[2]
        dbw_status = args[3]

        throttle = self.pid_velocity.step(proposed_linear_velocity - current_linear_velocity, 0.02)

        return throttle, 0., 0.

    def reset(self):
        self.pid_velocity.reset()