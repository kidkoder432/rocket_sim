from numpy import inf
def sign(x):
    if x == 0:
        return 1
    else:
        return x / abs(x)

class PID:

    def __init__(self, Kp, Ki, Kd, N=0, setpoint=0, dt=0):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.N = N
        self.alpha = 0

        self.setpoint = setpoint
        self.dt = dt

        self.integral = 0
        self.error = 0
        self.last_error = 0
        self.filtered_error = 0
        self.last_filter = 0

        self.limits = (-5, 5)
        self.components = ()

    def __call__(self, current_value, dt=None):

        if dt is not None:
            self.dt = dt
        self.error = self.setpoint - current_value


        if self.N <= 0:
            derivative = (self.error - self.last_error) / self.dt
        else:

            alpha = self.dt * self.N

            b = alpha / (alpha + 2)

            
            self.filtered_error = (
                b * (self.error + self.last_error) + (1 - b * 2) * self.last_filter
            )
            derivative = (self.filtered_error - self.last_filter) / self.dt

        output = self.Kp * self.error + self.Ki * self.integral + self.Kd * derivative

        self.components = (
            self.Kp * self.error,
            self.Ki * self.integral,
            self.Kd * derivative,
        )

        
        if self.doIntegratorClamp(sum(self.components)):
            self.integral += 0
        else:
            self.integral += self.error * self.dt
        self.last_error = self.error
        self.last_filter = self.filtered_error
        
        return max(self.limits[0], min(output, self.limits[1]))


    def reset(self):
        self.integral = 0
        self.last_error = 0
        self.last_filter = 0

    def doIntegratorClamp(self, out):
        saturated = out < self.limits[0] or out > self.limits[1]
        sameSign = sign(out) == sign(self.error)

        return saturated and sameSign

