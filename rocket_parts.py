import numpy as np
from collections import deque
import os
from csv import reader
from scipy.interpolate import InterpolatedUnivariateSpline

DEFAULT_GIMBAL_CONFIG = {
    "enable_servo_delay": True,
    "servo_delay_time": 0.04,
    "max_gimbal_deg": 5,
    "rate_limit": 500,
    "precision": 0.5,
}

DEFAULT_ENGINE_CONFIG = {
    "thrust_data": "f15.csv",
    "initial_mass": 0.1018,
    "final_mass": 0.0418,
    "burn_time": 3.45,
}

DEFAULT_STRUCTURE_CONFIG = {
    "dry_mass": 0.959 - 0.2036,
    "wet_mass": 0.959,
    "x_cg": 0.846,
    "x_t": 1.05,
    "moi_initial": 0.0739,
    "frontal_area": 0.00434,
}


class Gimbal:

    def reset(self, config=DEFAULT_GIMBAL_CONFIG):

        for key, value in config.items():
            setattr(self, key, value)

        self.current_gimbal_angle_deg = 0
        self.command_queue = deque()

        self.last_time = None

        self.allowed_angles = np.arange(-self.max_gimbal_deg, self.max_gimbal_deg, self.precision)


    def __init__(self, config=DEFAULT_GIMBAL_CONFIG):
        

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

        idx = np.argmin(np.abs(self.allowed_angles - gimbal_command_deg))
        self.current_gimbal_angle_deg = self.allowed_angles[idx]

        return self.current_gimbal_angle_deg

    def get_deg(self):
        return self.current_gimbal_angle_deg

    def get_rad(self):
        return np.radians(self.current_gimbal_angle_deg)


class Engine:

    def __init__(self, config=DEFAULT_ENGINE_CONFIG):

        for key, value in config.items():
            setattr(self, key, value)

        self.mass = self.initial_mass
        self.thrust = 0

        self._load_thrust_data(self.thrust_data)

    def _load_thrust_data(self, filename):
        """Loads thrust data and creates an interpolator."""
        # (Same as previous version)
        self.thrust_time_points, self.thrust_force_points = [], []
        if not os.path.exists(filename):
            print(f"Error: Thrust file '{filename}' not found.")
        else:
            try:
                with open(filename, "r") as f:
                    csv_reader = reader(f)
                    data = list(csv_reader)
                points = [(float(x[0]), float(x[1])) for x in data if len(x) >= 2]
                self.thrust_time_points = np.array([p[0] for p in points])
                self.thrust_force_points = np.array([p[1] for p in points])
                if len(self.thrust_time_points) == 0 or not np.all(
                    np.diff(self.thrust_time_points) > 0
                ):
                    print(f"Warning: Thrust data '{filename}' bad.")
                    self.thrust_time_points, self.thrust_force_points = [], []
            except Exception as e:
                print(f"Error reading thrust file '{filename}': {e}")
                self.thrust_time_points, self.thrust_force_points = [], []
        if len(self.thrust_time_points) > 0:
            print("Thrust data:")
            print("Time (s)\tThrust (N)")
            for t, f in zip(self.thrust_time_points, self.thrust_force_points):
                print(f"{t:.2f}\t{f:.2f}")
            print()
            self._thrust_interpolator = InterpolatedUnivariateSpline(
                self.thrust_time_points,
                self.thrust_force_points,
            )
            print(f"Thrust data loaded from '{filename}'.")
        else:
            print("Warning: Using dummy thrust (0).")
            self._thrust_interpolator = lambda t: 0.0

    def _thrust_at_time(self, t):
        """Calculates thrust at a given time."""
        if t <= 0 or t >= self.thrust_time_points[-1]:
            return 0.0
        return self._thrust_interpolator(t)

    def _engine_mass_at_time(self, t, initial_mass, final_mass, burn_time):
        """Calculates the mass of a single engine at time t during its burn."""
        # (Same as previous version)
        if t < 0:
            return initial_mass
        if t >= burn_time:
            return final_mass
        if burn_time <= 0:
            return final_mass
        return initial_mass + (final_mass - initial_mass) * (t / burn_time)

    def update(self, t):
        self.thrust = self._thrust_at_time(t)
        self.mass = self._engine_mass_at_time(t, self.initial_mass, self.final_mass, self.burn_time)

class Structure:
    def __init__(self, config=DEFAULT_STRUCTURE_CONFIG):
        for key, value in config.items():
            setattr(self, key, value)

        self.reset()

    def reset(self):
        self.mass = self.wet_mass
        engine_mass = self.wet_mass - self.dry_mass

        self.moment_arm = abs(self.x_t - self.x_cg)
        self.cg_dry = (self.x_cg * self.mass - self.x_t * engine_mass) / self.dry_mass

        self.moi = self.moi_initial
        self.moi_dry = (
            self.moi_initial
            - (self.x_cg - self.x_t) ** 2 * engine_mass
            - (self.x_cg - self.cg_dry) ** 2 * self.dry_mass
        )

    def update(self, e1_mass, e2_mass):

        total_engine_mass = e1_mass + e2_mass
        self.mass = self.dry_mass + total_engine_mass

        self.current_cg = (self.dry_mass * self.cg_dry + total_engine_mass * self.x_t) / self.mass

        self.moment_arm = abs(self.x_t - self.current_cg)

        self.moi = (self.moi_dry
            + (self.current_cg - self.cg_dry)**2 * self.dry_mass
            + (self.current_cg - self.x_t)**2 * total_engine_mass
        )

        return self.mass, self.moment_arm, self.moi
