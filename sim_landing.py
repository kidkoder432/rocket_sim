import sim_final
from pid import PID
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from multiprocessing import Pool
import itertools
from tqdm import tqdm


# --- Configuration & Global State ---
# These will be initialized ONCE per worker process
worker_sim = None
worker_controller = None


def init_worker():
    """
    Initializes the simulator and controller exactly once per CPU core.
    This prevents the overhead of object creation for every simulation run.
    """
    global worker_sim, worker_controller
    worker_sim = sim_final.RocketSimulator()
    worker_controller = PID(
        Kp=worker_sim.config["pid_kp"],
        Ki=worker_sim.config["pid_ki"],
        Kd=worker_sim.config["pid_kd"],
        N=-1,
        setpoint=worker_sim.config["pid_setpoint"],
        dt=worker_sim.config["delta_time"],
        limits=worker_sim.config["gimbal_limits"],
    )


def run_sim_task(params):
    """
    The core simulation loop. Optimized to avoid DataFrame overhead
    during the high-speed execution.
    """
    alt, mass = params

    # Reset existing objects instead of creating new ones
    current_state = worker_sim.reset()
    worker_sim.set_target_burn_alt(alt)
    worker_sim.set_initial_mass(mass)
    worker_controller.reset()

    done = False
    last_velocity_y = 0.0

    while not done:
        # Get command and step physics
        gimbal_cmd = worker_controller.update(current_state)
        next_state, done = worker_sim.step(gimbal_cmd)

        # Update state for next iteration
        current_state = next_state

        done = worker_sim.done()

    last_velocity_y = worker_sim.get_current_state()["velocity_y"]
    # Return only the necessary scalar values to minimize inter-process communication
    return (last_velocity_y, worker_sim.time, worker_sim.stage2_ignition_time)


if __name__ == "__main__":
    # 1. Define your axes
    alts = np.arange(30, 50, 1)
    masses = np.arange(950, 1000, 1) / 1000

    # 2. Generate the Cartesian Product (Every Alt vs Every Mass)
    # This creates the full grid required for a Heatmap
    param_grid = list(itertools.product(alts, masses))

    print(f"Starting simulation of {len(param_grid)} iterations...")

    # 3. Execute in Parallel
    # 'initializer' ensures init_worker() runs when a process starts

    with Pool(processes=4, initializer=init_worker) as pool:
        results = list(tqdm(pool.imap(run_sim_task, param_grid), total=len(param_grid)))

    # 4. Reshape the flat results list back into 2D grids
    # Results are (vel, total_time, ignition_time)
    lvels_data = np.array([res[0] for res in results]).reshape((len(alts), len(masses)))

    print("Simulation complete. Generating plot...")

    # 5. Visualization
    fig = go.Figure(
        data=[
            go.Heatmap(
                z=lvels_data,
                x=masses,
                y=alts,
                colorscale="Viridis",
                colorbar=dict(title="Landing Velocity (m/s)"),
            )
        ]
    )

    fig.update_layout(
        title="Stability Map: Altitude vs Mass vs Landing Velocity",
        xaxis_title="Initial Mass (kg)",
        yaxis_title="Target Burn Altitude (m)",
        template="plotly_dark",
        xaxis=dict(gridcolor="gray", zerolinecolor="white"),
        yaxis=dict(gridcolor="gray", zerolinecolor="white"),
    )

    fig.show()
