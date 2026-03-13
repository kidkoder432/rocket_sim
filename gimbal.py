import numpy as np
from collections import deque

DEFAULT_CONFIG = {
    "enable_servo_delay": True,
    "servo_delay_time": 0.04,
    "max_gimbal_deg": 5,
    "rate_limit": 500,
}


class Gimbal:

    def reset(self, config=DEFAULT_CONFIG):

        for key, value in config.items():
            setattr(self, key, value)

        self.current_gimbal_angle_deg = 0
        self.command_queue = deque()

        self.last_time = None

    def __init__(self, config=DEFAULT_CONFIG):

        self.reset(config)

    def __call__(self, gimbal_command_deg: float, time):

        # initialize dt
        if self.last_time is None:
            self.last_time = time

        dt = time - self.last_time
        self.last_time = time

        # --- servo delay ---
        if self.enable_servo_delay:

            self.command_queue.append((gimbal_command_deg, time))

            if (
                self.command_queue
                and time - self.command_queue[0][1] >= self.servo_delay_time
            ):
                gimbal_command_deg, _ = self.command_queue.popleft()
            else:
                gimbal_command_deg = self.current_gimbal_angle_deg

        # --- angle saturation ---
        gimbal_command_deg = np.clip(
            gimbal_command_deg, -self.max_gimbal_deg, self.max_gimbal_deg
        )

        # --- rate limiting ---
        max_step = self.rate_limit * dt

        delta = gimbal_command_deg - self.current_gimbal_angle_deg
        delta = np.clip(delta, -max_step, max_step)

        self.current_gimbal_angle_deg += delta
        return self.current_gimbal_angle_deg

    def get_deg(self):
        return self.current_gimbal_angle_deg

    def get_rad(self):
        return np.radians(self.current_gimbal_angle_deg)
