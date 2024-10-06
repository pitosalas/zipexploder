import numpy as np


class PID:
    def __init__(self, min_val, max_val, kp, ki, kd, integral_limit=None):
        self.min_val = min_val
        self.max_val = max_val
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0
        self.derivative = 0
        self.integral_limit = integral_limit
        

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value

        self.integral += error
        if self.integral_limit is not None:
            self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)

        derivative_smoothing_factor = 0.1 
        derivative = error - self.prev_error
        self.derivative = (self.derivative * derivative_smoothing_factor) + ((1 - derivative_smoothing_factor) * derivative)

        pid = (self.kp * error) + (self.ki * self.integral) + (self.kd * self.derivative)
        self.prev_error = error

        return np.clip(pid, self.min_val, self.max_val)
    
    # def clear(self):
    #     self.integral = 0
    #     self.prev_error = 0
    #     self.derivative = 02