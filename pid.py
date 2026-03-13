import numpy as np

# Import Dict and Any for type hinting the state dictionary
from typing import Tuple, Dict, Any


# Helper function (used in anti-windup)
def sign(x):
    """Returns the sign of a number (1 for positive/zero, -1 for negative)."""
    return 1 if x >= 0 else -1


class PID:
    """
    A PID controller class conforming to the state-dictionary Controller protocol.
    Handles its own output saturation and anti-windup internally.
    Uses derivative-on-measurement with filtering.
    """

    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        N: float,
        setpoint: float,
        dt: float,
        limits: Tuple[float, float],
    ):
        """
        Initializes the PID controller.

        Args:
            Kp: Proportional gain.
            Ki: Integral gain.
            Kd: Derivative gain.
            N:  Derivative filter coefficient. Higher N means less filtering (more noise).
                N=0 disables the filter.
            setpoint: The target value for the controlled variable (e.g., 0 degrees angle).
            dt: The time step (seconds) between updates.
            limits: A tuple (min_output, max_output) for controller output saturation.
        """
        # Parameters
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.N = N  # Filter coefficient
        self.setpoint = setpoint  # Target value (in degrees)
        self.dt = dt
        self._limits = limits  # Internal storage for output limits

        # State variables - initialized in reset()
        self._integral: float = 0.0
        self._derivative: float = 0.0  # Filtered derivative state
        self._previous_measurement_deg: float = 0.0
        self._previous_error: float = 0.0

        # Optional: Store last components for debugging/logging
        self.last_components: Tuple[float, float, float] = (0.0, 0.0, 0.0)

        # Initialize states
        self.reset()

        # Pre-calculate filter alpha if possible (only if N and dt are constant and positive)
        self._alpha_filter = 1.0  # Default if no filter
        if self.N > 0 and self.dt > 0:
            Tf = self.Kd / (self.Kp * self.N) if self.Kp != 0 else self.dt / 2
            self._alpha_filter = self.dt / (Tf + self.dt) if (Tf + self.dt) > 0 else 1.0

    # *** UPDATED method signature and input handling ***
    def update(self, state: Dict[str, Any]) -> float:
        """
        Calculates PID output based on the state dictionary.
        This implementation primarily uses 'theta_measured_radians' from the state.

        Args:
            state: Dictionary containing the full simulation state.
                   Expected keys include 'theta_measured_radians'.

        Returns:
            np.array([commanded_gimbal_angle_deg]) - saturated output.
        """
        # --- Input Processing ---
        # Extract relevant state using dictionary key - Use .get for safety
        measured_angle_rad = state.get("theta_measured_radians", 0.0)
        measurement_deg = np.degrees(measured_angle_rad)

        # Calculate error
        error = self.setpoint - measurement_deg

        # --- Proportional Term ---
        P = self.Kp * error

        # --- Integral Term (calculated before anti-windup adjustment) ---
        I = self.Ki * self._integral

        # --- Derivative Term (on measurement, with filtering) ---
        if self.dt > 0:
            raw_derivative_meas = (
                measurement_deg - self._previous_measurement_deg
            ) / self.dt
            if self.N > 0:
                self._derivative = (
                    self._alpha_filter * raw_derivative_meas
                    + (1 - self._alpha_filter) * self._derivative
                )
            else:
                self._derivative = raw_derivative_meas
            D = self.Kd * (self._derivative)
        else:
            D = 0.0

        # --- Calculate Pre-Saturated Output ---
        output = P + I + D
        self.last_components = (P, I, D)

        # --- Apply Saturation Internally ---
        output_saturated = np.clip(output, self._limits[0], self._limits[1])

        # --- Anti-windup (Integrator Clamping) ---
        is_saturated = output < self._limits[0] or output > self._limits[1]
        error_pushes_further = sign(output) == sign(error)

        if not (is_saturated and error_pushes_further):
            if self.Ki != 0 and self.dt > 0:
                self._integral += error * self.dt

        # --- Update States for Next Iteration ---
        self._previous_error = error
        self._previous_measurement_deg = measurement_deg

        # --- Return Control Output ---
        return -output_saturated

    def reset(self):
        """Resets the integral and derivative states."""
        self._integral = 0.0
        self._derivative = 0.0
        self._previous_error = 0.0
        self._previous_measurement_deg = 0.0
        self.last_components = (0.0, 0.0, 0.0)
