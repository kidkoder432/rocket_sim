import numpy as np
from scipy.interpolate import interp1d
from csv import reader
import os
from pid import PID
from gimbal import Gimbal
from vpython import *

# Added Protocol for the example usage block's type hint
from typing import Tuple, Dict, Any, Optional, List, Protocol


# Helper function
def sign(x):
    """Returns the sign of a number (1 for positive/zero, -1 for negative)."""
    return 1 if x >= 0 else -1


# ==============================================================
# --- Configuration & Constants ---
# ==============================================================
# (Constants remain the same as previous version)
# --- File Paths ---
THRUST_DATA_FILE = "f15.csv"
OUTPUT_DATA_FILE = "simulation_data.csv"
# --- Simulation Control ---
SIMULATION_DURATION = 30
DELTA_TIME = 0.01
# --- Physical Constants ---
GRAVITY = 9.80665
AIR_DENSITY = 1.293
DRAG_COEFFICIENT = 1.14
FRONTAL_AREA = 0.00434
# --- Rocket Parameters ---
INITIAL_MASS_TOTAL = 0.834
ENGINE1_INITIAL_MASS = 0.1018
ENGINE1_FINAL_MASS = 0.0418
ENGINE1_BURN_TIME = 3.45
ENGINE2_INITIAL_MASS = ENGINE1_INITIAL_MASS
ENGINE2_FINAL_MASS = ENGINE1_FINAL_MASS
ENGINE2_BURN_TIME = ENGINE1_BURN_TIME
STRUCTURE_MASS = INITIAL_MASS_TOTAL - ENGINE1_INITIAL_MASS - ENGINE2_INITIAL_MASS
if STRUCTURE_MASS < 0:
    print(f"Warning: Calculated STRUCTURE_MASS ({STRUCTURE_MASS:.4f} kg) is negative.")
    STRUCTURE_MASS = 0
# --- Staging Parameters ---
TARGET_STAGE2_IGNITION_ALTITUDE = 82
STAGE2_IGNITION_WINDOW = 0.4
# --- Initial Conditions ---
LAUNCH_ANGLE_DEG = 15.0
# --- Control System Parameters ---
KP = 1.44168417
KI = 2.64033837
KD = -0.14789756
N_FILTER = 70
PID_SETPOINT = 0.0
PID_OUTPUT_LIMITS = (-5, 5)
# --- Moment Arms & Inertia ---
MOMENT_ARM_STAGE1 = 0.29
MOI_STAGE1 = 0.0739
MOMENT_ARM_STAGE2 = 0.302
MOI_STAGE2 = 0.0612
# --- Simulation Options ---
ENABLE_SENSOR_NOISE = True
SENSOR_NOISE_STD_DEV = 0.1
ENABLE_SERVO_DELAY = True
SERVO_DELAY_TIME = 0.04
\

class RocketSimulator:
    """
    A general-purpose, class-based 2D rocket simulator.

    Manages the simulation state and physics. Can be stepped through time
    by providing control inputs (gimbal angle). Designed for behavioral
    alignment with the procedural version when run via the example usage loop.
    Suitable for use with various external controllers (PID, RL, State-Space, etc.).
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initializes the Rocket Simulator.

        Args:
            config: An optional dictionary containing configuration parameters.
                    If None, uses default values. User-provided keys override defaults.
        """
        # --- Load Configuration ---
        default_config = self._get_default_config()
        # Merge user config with defaults, ensuring all keys from default exist
        user_config = config if config is not None else {}
        self.config = {
            **default_config,
            **user_config,
        }  # User config overrides defaults

        self.gimbal = Gimbal()

        # --- Simulation Control ---
        self.dt = self.config["delta_time"]
        self.max_duration = self.config[
            "simulation_duration"
        ]  # Max duration for external loops

        # --- Physical Constants & Rocket Params ---
        self.g = self.config["gravity"]
        self.rho = self.config["air_density"]
        self.cd = self.config["drag_coefficient"]
        self.frontal_area = self.config["frontal_area"]
        self.initial_mass_total = self.config["initial_mass_total"]
        self.eng1_initial_mass = self.config["engine1_initial_mass"]
        self.eng1_final_mass = self.config["engine1_final_mass"]
        self.eng1_burn_time = self.config["engine1_burn_time"]
        self.eng2_initial_mass = self.config["engine2_initial_mass"]
        self.eng2_final_mass = self.config["engine2_final_mass"]
        self.eng2_burn_time = self.config["engine2_burn_time"]
        # Structure mass is calculated, not configured directly
        self.structure_mass = (
            self.initial_mass_total - self.eng1_initial_mass - self.eng2_initial_mass
        )
        if self.structure_mass < 0:
            print(
                f"Warning: Calculated STRUCTURE_MASS ({self.structure_mass:.4f} kg) is negative."
            )
            self.structure_mass = 0

        # --- Staging ---
        self.target_stage2_alt = self.config["target_stage2_ignition_altitude"]
        self.stage2_alt_window = self.config["stage2_ignition_window"]

        # --- Initial & Control ---
        self.launch_angle_deg = self.config["launch_angle_deg"]
        self.min_gimbal_angle = self.config["gimbal_limits"][0]
        self.max_gimbal_angle = self.config["gimbal_limits"][1]

        # --- Dynamics Params ---
        self.moment_arm_stage1 = self.config["moment_arm_stage1"]
        self.moi_stage1 = self.config["moi_stage1"]
        self.moment_arm_stage2 = self.config["moment_arm_stage2"]
        self.moi_stage2 = self.config["moi_stage2"]

        # --- Load Thrust Data ---
        self._load_thrust_data(self.config["thrust_data_file"])

        # --- Internal State Variables (initialized in reset) ---
        self.time: float = 0.0
        self.altitude: float = 0.0
        self.x_position: float = 0.0
        self.velocity_x: float = 0.0
        self.velocity_y: float = 0.0
        self.theta_radians: float = 0.0  # Actual angle
        self.angular_velocity: float = 0.0  # rad/s
        self.angular_acceleration: float = 0.0  # rad/s^2 (calculated in step)
        self.current_gimbal_angle_deg: float = 0.0  # Actual angle after delay
        self.command_queue: List[Tuple[float, float]] = []
        self.stage1_separation_time: float = self.eng1_burn_time
        self.stage2_ignition_time: float = np.inf
        self.stage2_burnout_time: float = np.inf
        self.stage2_ignition_altitude: float = 0.0
        self.stage2_ignition_locked: bool = False
        self.current_stage: float = 1.0
        self.current_mass: float = 0.0
        self.current_moi: float = 0.0
        self.current_moment_arm: float = 0.0
        self.engine_thrust: float = 0.0
        self.is_burning: bool = False
        self.drag_force_x: float = 0.0  # Store last calculated drag
        self.drag_force_y: float = 0.0  # Store last calculated drag
        self.aoa: float = 0.0  # Angle of attack in degrees

        print("General RocketSimulator initialized.")

    def _get_default_config(self) -> Dict[str, Any]:
        """Returns a dictionary with default simulation parameters."""
        # This dictionary now matches the procedural script's constants
        return {
            # --- Simulation Control ---
            "thrust_data_file": THRUST_DATA_FILE,
            "simulation_duration": SIMULATION_DURATION,
            "delta_time": DELTA_TIME,
            # --- Physical Constants ---
            "gravity": GRAVITY,
            "air_density": AIR_DENSITY,
            "drag_coefficient": DRAG_COEFFICIENT,
            "frontal_area": FRONTAL_AREA,
            # --- Rocket Parameters ---
            "initial_mass_total": INITIAL_MASS_TOTAL,
            "engine1_initial_mass": ENGINE1_INITIAL_MASS,
            "engine1_final_mass": ENGINE1_FINAL_MASS,
            "engine1_burn_time": ENGINE1_BURN_TIME,
            "engine2_initial_mass": ENGINE2_INITIAL_MASS,
            "engine2_final_mass": ENGINE2_FINAL_MASS,
            "engine2_burn_time": ENGINE2_BURN_TIME,
            # --- Staging Parameters ---
            "target_stage2_ignition_altitude": TARGET_STAGE2_IGNITION_ALTITUDE,
            "stage2_ignition_window": STAGE2_IGNITION_WINDOW,
            # --- Initial Conditions ---
            "launch_angle_deg": LAUNCH_ANGLE_DEG,
            # --- Control System Parameters ---
            "pid_kp": KP,
            "pid_ki": KI,
            "pid_kd": KD,
            "pid_n": N_FILTER,
            "pid_setpoint": PID_SETPOINT,
            "gimbal_limits": PID_OUTPUT_LIMITS,
            # --- Moment Arms & Inertia ---
            "moment_arm_stage1": MOMENT_ARM_STAGE1,
            "moi_stage1": MOI_STAGE1,
            "moment_arm_stage2": MOMENT_ARM_STAGE2,
            "moi_stage2": MOI_STAGE2,
            # --- Simulation Options ---
            "enable_servo_delay": ENABLE_SERVO_DELAY,
            "servo_delay_time": SERVO_DELAY_TIME,
            "enable_sensor_noise": ENABLE_SENSOR_NOISE,
            "sensor_noise_std_dev": SENSOR_NOISE_STD_DEV,
        }

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
            self._thrust_interpolator = interp1d(
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

    def reset(self) -> Dict[str, Any]:
        """
        Resets the simulation environment to its initial state based on config.

        Returns:
            Dict[str, Any]: The initial state dictionary.
        """
        self.time = 0.0
        self.altitude = 0.0
        self.x_position = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.theta_radians = np.radians(self.config["launch_angle_deg"])
        self.angular_velocity = 0.0
        self.angular_acceleration = 0.0
        self.current_gimbal_angle_deg = 0.0
        self.command_queue = []
        self.stage1_separation_time = self.config["engine1_burn_time"]  # Use config
        self.stage2_ignition_time = np.inf
        self.stage2_burnout_time = np.inf
        self.stage2_ignition_altitude = 0.0
        self.stage2_ignition_locked = False
        self.current_stage = 1.0
        self.current_mass = self.config["initial_mass_total"]
        self.current_moi = self.config["moi_stage1"]
        self.current_moment_arm = self.config["moment_arm_stage1"]
        self.engine_thrust = 0.0  # Will be calculated in first step if t=0
        self.is_burning = False  # Will be determined in first step
        self.drag_force_x = 0.0
        self.drag_force_y = 0.0
        self.aoa = 0.0

        self.gimbal.reset()

        # print(f"Environment reset. Initial angle: {np.degrees(self.theta_radians):.1f} deg") # Optional
        return self.get_current_state()

    def get_current_state(self) -> Dict[str, Any]:
        """
        Returns the current state of the simulation as a dictionary.
        Includes a 'theta_measured_radians' key for convenience, applying noise
        based on the configuration.
        """
        # Apply sensor noise based on config
        noise_std_dev_deg = self.config.get("sensor_noise_std_dev", 0.1)
        enable_noise = self.config.get("enable_sensor_noise", True)

        noise = (
            np.random.normal(0, np.radians(noise_std_dev_deg)) if enable_noise else 0.0
        )
        theta_measured_radians = self.theta_radians + noise

        return {
            "time": self.time,
            "altitude": self.altitude,
            "x_position": self.x_position,
            "velocity_x": self.velocity_x,
            "velocity_y": self.velocity_y,
            "theta_radians": self.theta_radians,  # Actual angle
            "theta_measured_radians": theta_measured_radians,  # Noisy angle for controllers
            "angular_velocity": self.angular_velocity,
            "angular_acceleration": self.angular_acceleration,  # From previous step's calculation
            "current_mass": self.current_mass,
            "current_moi": self.current_moi,
            "current_moment_arm": self.current_moment_arm,
            "current_stage": self.current_stage,
            "engine_thrust": self.engine_thrust,  # Thrust applied in the *last* step
            "is_burning": self.is_burning,
            "drag_force_x": self.drag_force_x,
            "drag_force_y": self.drag_force_y,  # Drag calculated in the *last* step
            "angle_of_attack_deg": self.aoa,
            "gimbal_actual_deg": self.current_gimbal_angle_deg,  # Gimbal angle applied in the *last* step
            "dt": self.dt,
        }

    def _overtime(self) -> bool:
        """
        Checks if the simulation has reached a terminal state.

        Returns:
            bool: True if the simulation is done, False otherwise.
        """
        return self.time >= self.max_duration

    def _crashed(self) -> bool:
        """
        Checks if the rocket has crashed (altitude below -1m after 1s).

        Returns:
            bool: True if the rocket has crashed, False otherwise.
        """
        return self.altitude < 0.0 and self.time > 1.0

    def done(self) -> bool:
        """
        Checks if the simulation has reached a terminal state (overtime or crash).

        Returns:
            bool: True if the simulation is done, False otherwise.
        """
        return self._overtime() or self._crashed()

    def step(self, gimbal_command_deg: float) -> Tuple[Dict[str, Any], bool]:
        """
        Runs one time step of the simulation using the provided gimbal command.

        Args:
            gimbal_command_deg: The desired gimbal angle command in degrees.

        Returns:
            Dict[str, Any]: The dictionary representing the new state after the step.
        """

        self.gimbal(gimbal_command_deg, self.time)
        gimbal_radians = self.gimbal.get_rad()
        self.current_gimbal_angle_deg = self.gimbal.get_deg()

        # --- 2. Update Stage & Dynamics ---
        # Determine current stage, mass, moi, moment_arm, thrust, burning status
        # This logic is identical to the procedural version.
        self.is_burning = False
        self.engine_thrust = 0.0
        if self.time < self.stage1_separation_time:
            self.current_stage = 1.0
            engine_time = self.time
            self.engine_thrust = self._thrust_at_time(engine_time)
            engine1_mass = self._engine_mass_at_time(
                engine_time,
                self.eng1_initial_mass,
                self.eng1_final_mass,
                self.eng1_burn_time,
            )
            self.current_mass = (
                self.structure_mass + engine1_mass + self.eng2_initial_mass
            )
            self.current_moi = self.moi_stage1
            self.current_moment_arm = self.moment_arm_stage1
            self.is_burning = True
        elif self.time < self.stage2_ignition_time:
            self.current_stage = 1.5
            self.current_mass = (
                self.structure_mass + self.eng1_final_mass + self.eng2_initial_mass
            )
            self.current_moi = self.moi_stage1
            self.current_moment_arm = self.moment_arm_stage1
            # Check stage 2 ignition (identical logic)
            if (
                not self.stage2_ignition_locked
                and self.velocity_y < 0
                and abs(self.altitude - self.target_stage2_alt)
                <= self.stage2_alt_window
            ):
                self.stage2_ignition_time = self.time
                self.stage2_ignition_altitude = self.altitude
                self.stage2_burnout_time = (
                    self.stage2_ignition_time + self.eng2_burn_time
                )
                self.stage2_ignition_locked = True
        elif self.time < self.stage2_burnout_time:
            self.current_stage = 2.0
            engine_time = self.time - self.stage2_ignition_time
            self.engine_thrust = self._thrust_at_time(engine_time)
            engine2_mass = self._engine_mass_at_time(
                engine_time,
                self.eng2_initial_mass,
                self.eng2_final_mass,
                self.eng2_burn_time,
            )
            self.current_mass = (
                self.structure_mass + self.eng1_final_mass + engine2_mass
            )
            self.current_moi = self.moi_stage2
            self.current_moment_arm = self.moment_arm_stage2
            self.is_burning = True
        else:
            self.current_stage = 2.5
            self.current_mass = (
                self.structure_mass + self.eng1_final_mass + self.eng2_final_mass
            )
            self.current_moi = self.moi_stage2
            self.current_moment_arm = self.moment_arm_stage2

        # Safety check for mass
        if self.current_mass <= 0:
            print(
                f"Error: Mass non-positive ({self.current_mass:.4f}) at t={self.time:.2f}. Step skipped."
            )
            # Return current state without advancing time if mass is invalid
            # This prevents division by zero in physics calcs
            return self.get_current_state()

        # --- 3. Calculate Physics ---
        # Use state determined *for this step* (thrust, mass, moi, moment_arm, gimbal_radians)
        thrust_force = self.engine_thrust
        self.speed = np.sqrt(self.velocity_x**2 + self.velocity_y**2)

        self.drag_force_x = (
            0.5
            * self.rho
            * ((self.velocity_x * self.speed) if self.speed > 0 else 0.0)
            * self.cd
            * self.frontal_area
        )

        self.drag_force_y = (
            0.5
            * self.rho
            * ((self.velocity_y * self.speed) if self.speed > 0 else 0.0)
            * self.cd
            * self.frontal_area
        )

        gravity_force_y = -self.g * self.current_mass
        total_angle_radians = (
            self.theta_radians + gimbal_radians
        )  # Angle of thrust vector in world frame
        force_x = thrust_force * np.sin(total_angle_radians) - self.drag_force_x
        force_y = (
            thrust_force * np.cos(total_angle_radians)
            + gravity_force_y
            + self.drag_force_y
        )
        acceleration_x = force_x / self.current_mass
        acceleration_y = force_y / self.current_mass
        torque = -thrust_force * np.sin(gimbal_radians) * self.current_moment_arm
        # Calculate angular acceleration *for this step*
        if self.current_moi <= 0:
            self.angular_acceleration = 0.0
        else:
            self.angular_acceleration = torque / self.current_moi

        self.move_angle = np.arctan2(self.velocity_y, self.velocity_x)
        self.aoa = np.degrees(self.theta_radians - self.move_angle) + 90

        if self.velocity_y <= 0:
            self.aoa = 0

        # --- 4. Integrate State ---
        # Update state variables based on accelerations calculated *this step*
        self.velocity_x += acceleration_x * self.dt
        self.velocity_y += acceleration_y * self.dt

        if acceleration_y < 0 and self.time < 1:
            self.velocity_y = max(0, self.velocity_y)

        self.x_position += self.velocity_x * self.dt
        self.altitude += self.velocity_y * self.dt
        self.angular_velocity += self.angular_acceleration * self.dt
        self.theta_radians += self.angular_velocity * self.dt
        # Advance time *after* all calculations for this step are done
        self.time += self.dt

        # --- 5. Return New State and Check for Done ---
        # The state dictionary reflects the results *after* this step's integration
        return self.get_current_state(), self.done()

    def set_target_burn_alt(self, target_altitude: float) -> None:
        self.target_stage2_alt = target_altitude

def plot_results(log: Dict[str, Any]):
    """
    Generates plots from the simulation log data using Plotly Subplots
    for better layout and scrolling.
    """
    if not log or not log["time"]:
        print("No data to plot.")
        return

    # --- Trajectory Plot (Separate Figure) ---
    fig_traj = go.Figure()
    fig_traj.add_trace(
        go.Scatter(
            x=log.get("x_position", []),
            y=log.get("altitude", []),
            mode="lines",
            name="Trajectory",
        )
    )
    fig_traj.update_layout(
        title="Rocket Trajectory (Altitude vs Downrange)",
        xaxis_title="X Position (m)",
        yaxis_title="Altitude (m)",
        template="plotly_dark",
        yaxis_range=[
            min(0, min(log.get("altitude", [0])) * 1.1),
            max(log.get("altitude", [1])) * 1.1 + 1,
        ],
        xaxis_range=[
            min(log.get("x_position", [0])) - 1,
            max(log.get("x_position", [0])) + 1,
        ],
        yaxis_scaleanchor="x",
        yaxis_scaleratio=1,
    )
    fig_traj.show()

    # --- Time Series Plots (Using Subplots) ---
    fig_time = make_subplots(
        rows=6,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.03,
        subplot_titles=(
            "Altitude (m)",
            "Vertical Velocity (m/s)",
            "Angles (deg)",
            "Thrust (N)",
            "Stage & Angular Velocity (rad/s)",
            "Mass (kg)",
        ),
    )
    # Row 1: Altitude
    fig_time.add_trace(
        go.Scatter(x=log["time"], y=log["altitude"], name="Altitude", showlegend=False),
        row=1,
        col=1,
    )
    # Row 2: Vertical Velocity
    fig_time.add_trace(
        go.Scatter(x=log["time"], y=log["velocity_y"], name="Vel Y", showlegend=False),
        row=2,
        col=1,
    )
    # Row 3: Angles
    fig_time.add_trace(
        go.Scatter(
            x=log["time"],
            y=np.degrees(log["theta_radians"]),
            name="Theta",
            line=dict(color="lightblue"),
        ),
        row=3,
        col=1,
    )
    fig_time.add_trace(
        go.Scatter(
            x=log["time"],
            y=log["gimbal_actual_deg"],
            name="Gimbal",
            line=dict(color="lightcoral", dash="dash"),
        ),
        row=3,
        col=1,
    )
    # Row 4: Thrust
    fig_time.add_trace(
        go.Scatter(
            x=log["time"], y=log["engine_thrust"], name="Thrust", showlegend=False
        ),
        row=4,
        col=1,
    )
    # Row 5: Stage & Angular Velocity
    fig_time.add_trace(
        go.Scatter(
            x=log["time"],
            y=log["current_stage"],
            name="Stage",
            line=dict(color="lightgreen"),
        ),
        row=5,
        col=1,
    )
    fig_time.add_trace(
        go.Scatter(
            x=log["time"],
            y=log["angular_velocity"],
            name="Ang Vel",
            line=dict(color="yellow", dash="dot"),
        ),
        row=5,
        col=1,
    )
    # Row 6: Mass
    fig_time.add_trace(
        go.Scatter(x=log["time"], y=log["current_mass"], name="Mass", showlegend=False),
        row=6,
        col=1,
    )

    fig_time.update_layout(
        title_text="Rocket Simulation Time Series",
        height=1500,
        template="plotly_dark",
        hovermode="x unified",
        legend=dict(
            traceorder="grouped",
            orientation="h",
            yanchor="bottom",
            y=1.01,
            xanchor="right",
            x=1,
        ),
    )
    fig_time.update_xaxes(showticklabels=False)  # Hide all first
    fig_time.update_xaxes(
        showticklabels=True, title_text="Time (s)", row=6, col=1
    )  # Show only bottom
    fig_time.show()


def vis_results(log: Dict[str, Any]):
    # 1. Setup Data & Scene
    # Assuming log is a dict of lists: log['time'][i] gives the time at frame i
    num_frames = len(log["time"])

    scene = canvas(
        title="Rocket Simulation Visualizer",
        width=800,
        height=600,
        background=color.black,
    )
    scene.userspin = False
    scene.autoscale = False

    # Remove perspective (important for 2D)
    scene.forward = vector(0, 0, -1)  # camera looking straight down z
    scene.up = vector(0, 1, 0)

    # 2. Geometry Setup
    rocket_length = 1
    rocket_radius = 0.037
    engine_length = 0.114

    # The rocket's position is its center of mass.
    rocket = cylinder(
        pos=vec(0, 0, 0),
        axis=vec(0, rocket_length, 0),
        radius=rocket_radius,
        color=color.white,
    )

    # The engine attaches to the bottom of the rocket.
    engine = cylinder(
        pos=vec(0, 0, 0),
        axis=vec(0, -engine_length, 0),
        radius=rocket_radius * 0.5,
        color=color.red,
    )

    ground = box(pos=vec(0, -0.11, 0), size=vec(100, 0.2, 100), color=color.green)

    # 3. State Management
    state = {
        "frame": 0,
        "running": False,
        "max_frame": num_frames - 1,
        "lock_view": True,
    }

    # 4. UI Callbacks
    def toggle_play(b):
        state["running"] = not state["running"]
        b.text = "Pause" if state["running"] else "Play"

    def reset_sim(b):
        state["running"] = False
        play_button.text = "Play"
        state["frame"] = 0
        time_slider.value = 0
        update_visuals(0)

    def scrub_time(s):
        state["running"] = False
        play_button.text = "Play"
        state["frame"] = int(s.value)
        update_visuals(state["frame"])

    def toggle_lock_view(c):
        state["lock_view"] = c.checked

    checkbox(text="Lock view to rocket", bind=toggle_lock_view, checked=True)

    scene.append_to_caption("  ")

    # 5. UI Elements
    scene.append_to_caption("\n")
    play_button = button(text="Play", bind=toggle_play)
    scene.append_to_caption("  ")
    button(text="Reset", bind=reset_sim)
    scene.append_to_caption("  Time: ")

    time_slider = slider(
        min=0, max=state["max_frame"], value=0, length=400, bind=scrub_time
    )

    scene.append_to_caption("\n\n")
    readout = wtext(text="Data will appear here.")

    # 6. Update Logic
    def update_visuals(f: int):
        # Extract current frame data
        t = log["time"][f]
        x = log["x_position"][f]
        y = log["altitude"][f]
        aoa = log["angle_of_attack_deg"][f]
        theta_rad = log["theta_radians"][f]
        gimbal_deg = log["gimbal_actual_deg"][f]

        # Rocket position (assuming data gives Center of Mass)
        rocket.pos = vec(x, y, 0)

        # Calculate rocket orientation vector (assuming 0 rad is pointing straight up)
        # If your 0 is along the x-axis, swap sin/cos accordingly.
        rocket_dir = vec(-np.sin(theta_rad), np.cos(theta_rad), 0)
        rocket.axis = rocket_dir * rocket_length

        # Calculate engine position (bottom of the rocket)
        # Shift back from the CoM by half the rocket's length
        engine.pos = rocket.pos

        # Calculate engine orientation (rocket angle + gimbal angle)
        gimbal_rad = np.radians(gimbal_deg)
        total_engine_angle = theta_rad + gimbal_rad
        engine_dir = vec(-np.sin(total_engine_angle), np.cos(total_engine_angle), 0)

        # Engine axis points downwards relative to its mounting point
        engine.axis = -engine_dir * engine_length

        if state["lock_view"]:
            z = scene.camera.pos.z
            scene.camera.pos = vec(x, y, z)

        # Update Readout
        data_str = f"<b>Frame:</b> {f} / {state['max_frame']} | "
        data_str += f"<b>Time:</b> {t:.2f} s\n"
        data_str += f"<b>Alt:</b> {y:.2f} m | <b>X:</b> {x:.2f} m\n"
        data_str += f"<b>Theta:</b> {np.degrees(theta_rad):.2f}° | <b>Gimbal:</b> {gimbal_deg:.2f}°\n"
        data_str += f"<b>Thrust:</b> {log['engine_thrust'][f]:.2f} N | <b>Mass:</b> {log['current_mass'][f]:.2f} kg\n"
        data_str += f"<b>Angle of Attack:</b> {aoa:.2f}°\n"
        data_str += f"<b>Drag Force X:</b> {log['drag_force_x'][f]:.2f} N | <b>Drag Force Y:</b> {log['drag_force_y'][f]:.2f} N"

        readout.text = data_str

    # 7. Main Loop
    update_visuals(0)  # Initialize first frame

    while True:
        rate(60)  # Target 60 FPS
        if state["running"]:
            if state["frame"] < state["max_frame"]:
                state["frame"] += 1
                time_slider.value = state["frame"]
                update_visuals(state["frame"])
            else:
                state["running"] = False
                play_button.text = "Play"


def save_results(log: Dict[str, Any], filename: str):
    """Saves key simulation data to a CSV file."""
    if not log or not log["time"]:
        print("No data to save.")
        return
    try:
        with open(filename, "w") as f:
            list_headers = [
                "time",
                "altitude",
                "x_position",
                "velocity_x",
                "velocity_y",
                "theta_radians",
                "theta_measured_radians",
                "angular_velocity",
                "angular_acceleration",
                "current_mass",
                "current_moi",
                "current_moment_arm",
                "current_stage",
                "engine_thrust",
                "is_burning",
                "drag_force_y",
                "gimbal_actual_deg",
                "dt",
            ]
            f.write(",".join(list_headers) + "\n")
            num_entries = len(log["time"])
            for i in range(num_entries):
                row_values = []
                for key in list_headers:
                    if key in log and i < len(log[key]):
                        value = log[key][i]
                        if isinstance(value, (int, float, np.number)):
                            row_values.append(f"{value:.4f}")
                        else:
                            row_values.append(str(value))
                    else:
                        row_values.append("")
                f.write(",".join(row_values) + "\n")
            if "summary" in log:
                f.write("\nSummary Stats\n")
                for key, value in log["summary"].items():
                    if isinstance(value, (int, float, np.number)):
                        f.write(f"{key},{value:.4f}\n")
                    else:
                        f.write(f"{key},{value}\n")
        print(f"Simulation data saved to '{filename}'")
    except Exception as e:
        print(f"Error saving data to '{filename}': {e}")


# --- Define Controller Protocol and PID locally for self-contained example ---
# (Same as previous version)
class Controller(Protocol):
    def update(self, state: Dict[str, Any]) -> np.ndarray: ...
    def reset(self): ...


import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# ==============================================================
# --- Example Usage (Full Simulation with External Controller) ---
# ==============================================================
if __name__ == "__main__":
    print("Running General Rocket Simulator Example...")

    # --- Configuration ---
    sim_config = {
        # --- Simulation Control ---
        "thrust_data_file": THRUST_DATA_FILE,
        "simulation_duration": SIMULATION_DURATION,
        "delta_time": DELTA_TIME,
        # --- Physical Constants ---
        "gravity": GRAVITY,
        "air_density": AIR_DENSITY,
        "drag_coefficient": DRAG_COEFFICIENT,
        "frontal_area": FRONTAL_AREA,
        # --- Rocket Parameters ---
        "initial_mass_total": INITIAL_MASS_TOTAL,
        "engine1_initial_mass": ENGINE1_INITIAL_MASS,
        "engine1_final_mass": ENGINE1_FINAL_MASS,
        "engine1_burn_time": ENGINE1_BURN_TIME,
        "engine2_initial_mass": ENGINE2_INITIAL_MASS,
        "engine2_final_mass": ENGINE2_FINAL_MASS,
        "engine2_burn_time": ENGINE2_BURN_TIME,
        # --- Staging Parameters ---
        "target_stage2_ignition_altitude": TARGET_STAGE2_IGNITION_ALTITUDE,
        "stage2_ignition_window": STAGE2_IGNITION_WINDOW,
        # --- Initial Conditions ---
        "launch_angle_deg": LAUNCH_ANGLE_DEG,
        # --- Control System Parameters ---
        "pid_kp": KP,
        "pid_ki": KI,
        "pid_kd": KD,
        "pid_n": N_FILTER,
        "pid_setpoint": PID_SETPOINT,
        "gimbal_limits": PID_OUTPUT_LIMITS,
        # --- Moment Arms & Inertia ---
        "moment_arm_stage1": MOMENT_ARM_STAGE1,
        "moi_stage1": MOI_STAGE1,
        "moment_arm_stage2": MOMENT_ARM_STAGE2,
        "moi_stage2": MOI_STAGE2,
        # --- Simulation Options ---
        "enable_servo_delay": ENABLE_SERVO_DELAY,
        "servo_delay_time": SERVO_DELAY_TIME,
        "enable_sensor_noise": ENABLE_SENSOR_NOISE,
        "sensor_noise_std_dev": SENSOR_NOISE_STD_DEV,
    }

    # --- Initialization ---
    simulator = RocketSimulator(config=sim_config)
    # Define PID parameters based on procedural script's constants
    controller = PID(
        Kp=simulator.config["pid_kp"],
        Ki=simulator.config["pid_ki"],
        Kd=simulator.config["pid_kd"],
        N=simulator.config["pid_n"],
        setpoint=simulator.config["pid_setpoint"],
        dt=simulator.config["delta_time"],
        limits=simulator.config["gimbal_limits"],
    )

    # controller = DummyController(20)

    # controller = BangBangController([-3, 3])

    # --- Simulation Loop ---
    current_state = simulator.reset()
    controller.reset()

    history = [current_state]  # Store history of states
    done = False
    info_history = []  # Store commands

    print(f"Running simulation for {simulator.max_duration} seconds...")
    while not done:
        # 1. Get command from controller
        control_vector = controller.update(current_state)
        gimbal_cmd = control_vector  # This is the *desired* command

        # Store command info *before* stepping
        info_history.append(
            {
                "time": current_state["time"],  # Time *before* this step
                "gimbal_command_deg": gimbal_cmd,  # Command generated based on current_state
                "gimbal_actual_deg": current_state[
                    "gimbal_actual_deg"
                ],  # Angle applied in *previous* step
            }
        )

        # 2. Step the simulator using the command
        next_state, done = simulator.step(gimbal_cmd)

        # 3. Log data (log the state *after* the step)
        history.append(next_state)

        # Check time at the *end* of the loop logic
        if simulator.done():
            print(f"INFO: Simulation finished at t={simulator.time:.2f}s")
            break  # Exit after processing the last step

        # 5. Update state for next iteration
        current_state = next_state
        # --- End Loop ---

    print(f"\nSimulation finished at t={simulator.time:.2f}s")
    final_state = history[-1]
    print(
        f"Final State: Alt={final_state['altitude']:.2f}m, VelY={final_state['velocity_y']:.2f}m/s, Angle={np.degrees(final_state['theta_radians']):.1f}deg"
    )

    # --- Plotting Example ---
    if history:

        df_state = pd.DataFrame(history)
        df_info = pd.DataFrame(info_history)

        plot_results(df_state.to_dict("list"))

    # --- Saving Example ---
    # save_results function would need to be defined or imported
    save_results(
        df_state.to_dict("list"), "simulation_output_class.csv"
    )  # Example saving state

    vis_results(df_state.to_dict("list"))  # Visualize with VPython
