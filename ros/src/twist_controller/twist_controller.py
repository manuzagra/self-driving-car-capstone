from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        self.pid = PID(['kp'], ['ki'], ['kd']) #, mn, mx) these numvers are for the saturation

        self.yaw_control = YawController(kwargs['wheel_base'],
                                         kwargs['steer_ratio'],
                                         kwargs['min_speed'],
                                         kwargs['max_lat_accel'],
                                         kwargs['max_steer_angle'])

        self.low_pass_filter = LowPassFilter(['tau'], ['ts'])

    def reset(self):
        self.pid.reset()
        self.low_pass_filter.reset()

    def control(self, reference, measured):
        # TwistStamped
        error_vel = reference.linear.x - measured.linear.x
        vel_control = self.pid(error_vel)
        # TODO probably this is not tthe correct way
        if vel_control > 0:
            throttle = vel_control
            brake = 0.
        else:
            throttle = 0.
            brake = -1*vel_control

        steer = self.yaw_controlget_steering(reference.linear.x, reference.angular.z, measured.linear.x)
        steer = self.low_pass_filter(steer)
        return throttle, brake, steer
