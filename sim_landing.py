import sim_final
from pid import PID
import numpy as np
import pandas as pd
from typing import Dict, Any
import plotly.graph_objects as go

alts = np.arange(81, 83, 0.01)

sim = sim_final.RocketSimulator()


# Define PID parameters based on procedural script's constants
controller = PID(
    Kp=sim.config["pid_kp"],
    Ki=sim.config["pid_ki"],
    Kd=sim.config["pid_kd"],
    N=sim.config["pid_n"],
    setpoint=sim.config["pid_setpoint"],
    dt=sim.config["delta_time"],
    limits=sim.config["gimbal_limits"],
)


def run_sim(alt: float) -> Dict[str, Any]:

    # --- Simulation Loop ---
    current_state = sim.reset()
    sim.set_target_burn_alt(alt)
    controller.reset()

    history = [current_state]  # Store history of states
    done = False

    while not done:
        # 1. Get command from controller
        control_vector = controller.update(current_state)
        gimbal_cmd = control_vector  # This is the *desired* command

        # 2. Step the simulator using the command
        next_state, done = sim.step(gimbal_cmd)

        # 3. Log data (log the state *after* the step)
        history.append(next_state)

        # Check time at the *end* of the loop logic
        if sim.done():
            # print(f"INFO: Simulation finished at t={sim.time:.2f}s")
            break  # Exit after processing the last step

        # 5. Update state for next iteration
        current_state = next_state
        # --- End Loop ---

    data = pd.DataFrame(history).to_dict(orient="list")

    return data["velocity_y"], data["time"][-1], sim.stage2_ignition_time

lvels = []

for alt in alts:

    vels, flight_time, on_time = run_sim(alt)
    print(
        f"Flight Time: {flight_time:.2f}s, Burn Altitude: {alt:.2f}m, Landing Velocity: {vels[-1]:.2f}m/s, Ignition Time: {on_time:.2f}s"
    )

    lvels.append(abs(vels[-1]))

fig = go.Figure(data=[go.Scatter(x=alts, y=lvels, mode="lines+markers", line=dict(color="white"), marker=dict(color="white", size=5))])
fig.update_layout(
    title="Altitude vs Landing Velocity",
    xaxis_title="Altitude (m)",
    yaxis_title="Landing Velocity (m/s)",
    template="plotly_dark",
    xaxis=dict(zeroline=True, zerolinecolor="white", gridcolor="gray", gridwidth=1, linecolor="white"),
    yaxis=dict(zeroline=True, zerolinecolor="white", gridcolor="gray", gridwidth=1, linecolor="white"),
)
fig.show()
