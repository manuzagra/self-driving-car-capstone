import time
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_error = 0.
        self.last_t = time.time() / 1000000000.

    def reset(self):
        self.int_val = 0.0
        self.last_t = time.time() / 1000000000.

    def step(self, error):
        t = time.time() / 1000000000.
        dt = t - self.last_t
        self.last_t = t

        integral = self.int_val + error * dt;
        derivative = (error - self.last_error) / dt;

        val = self.kp * error + self.ki * integral + self.kd * derivative;

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val
