
class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0, dt=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.dt = dt

        self.integral = 0
        self.last_error = 0

        self.limits = (-5, 5)
        self.components = ()

    def __call__(self, current_value):
        error = self.setpoint - current_value

        self.integral += error * self.dt
        derivative = (error - self.last_error) / self.dt

        self.last_error = error

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.components = (self.Kp * error, self.Ki * self.integral, self.Kd * derivative)

        output = max(self.limits[0], min(output, self.limits[1]))

        return output

    def reset(self):
        self.integral = 0
        self.last_error = 0
