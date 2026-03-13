"""
Reinforcement Learning Controller
"""

import numpy as np
import pandas as pd
from pprint import pprint
import plotly.graph_objects as go
from typing import Tuple, Dict, Any
from sim_final import RocketSimulator, plot_results, save_results

from pid import PID

GIMBAL_DEG = 5


class RLCPID(PID):
    def __init__(self, theta):
        self.theta = theta

        super().__init__(
            theta[0],
            theta[1],
            theta[2],
            N=-1,
            setpoint=0,
            dt=0.01,
            limits=(-GIMBAL_DEG, GIMBAL_DEG),
        )


class RocketSimRL:
    def __init__(self, rocket_simulator: RocketSimulator) -> None:
        self.rocket_simulator = rocket_simulator
        self.episode_count = 0

        self.prev_action = 0

    def reset(self, test=False) -> Dict[str, Any]:
        self.episode_count += 1

        # Gradually increase max angle up to 25 degrees
        max_angle = min(5 + 0.05 * self.episode_count, 25)

        # Sample angle from [-max_angle, max_angle]
        angle = np.random.uniform(-max_angle, max_angle)

        current_config = self.rocket_simulator.config

        current_config["launch_angle_deg"] = angle

        if test:
            angle = 15
            # angle = (
            #     np.random.uniform(-25, -10)
            #     if np.random.random() < 0.5
            #     else np.random.uniform(10, 25)
            # )
            current_config["launch_angle_deg"] = angle

        self.rocket_simulator.config = current_config

        self.prev_action = 0

        return self.rocket_simulator.reset()

    def step(self, action: float) -> Tuple[Dict[str, Any], float, bool, Dict[str, Any]]:
        state, done = self.rocket_simulator.step(action)

        loss = (
            10 * state["theta_measured_radians"]**2
            + 5 * abs(state["angular_velocity"])**2
            + 100 * abs(action - self.prev_action)
        )

        # Penalize being pinned at the limits
        if abs(action) >= (GIMBAL_DEG - 0.1):
            loss *= 2.0

        self.prev_action = action

        return state, loss, done, {}

    def _get_state(self):
        return np.array(self.rocket_simulator.get_current_state().values())


class Cubic(object):
    def __init__(self, a, b, c, d):
        self.a = a
        self.b = b
        self.c = c
        self.d = d

    def __call__(self, x):
        return self.a * x**3 + self.b * x**2 + self.c * x + self.d


class LinearRLController(object):
    def __init__(self, theta):
        self.theta = theta

    def update(self, state):
        vec = np.array(
            [
                np.clip(state["theta_measured_radians"] / np.pi, -1, 1),
                np.clip(state["angular_velocity"], -1, 1),
                np.clip(state["engine_thrust"] / 20, 0, 1),
            ]
        )
        return np.clip(np.dot(self.theta, vec), -GIMBAL_DEG, GIMBAL_DEG)

    def reset(self):
        pass


class CubicRLController(object):
    def __init__(self, theta):
        self.theta = theta
        self.c1 = Cubic(*theta[:4])
        self.c2 = Cubic(*theta[4:8])
        self.c3 = Cubic(*theta[8:])

    def update(self, state):

        th = state["theta_measured_radians"]
        aa = state["angular_velocity"]
        tf = state["engine_thrust"]

        result = np.clip(
            self.c1(th) + self.c2(aa) + self.c3(tf), -GIMBAL_DEG, GIMBAL_DEG
        )

        return result

    def reset(self):

        pass


import cma


def evaluate(theta, sim: RocketSimRL):
    controller = RLCPID(theta)
    state = sim.reset()
    controller.reset()

    loss = 0
    done = False
    while not done:
        action = controller.update(state)

        state, new_loss, done, _ = sim.step(action)
        loss += new_loss

    return loss


if __name__ == "__main__":
    sim = RocketSimulator()
    env = RocketSimRL(sim)

    es = cma.CMAEvolutionStrategy(
        [1, 0, 0],
        0.5,
        {
            "bounds": [-5, 5],
            "popsize": 16,
            "CMA_active": True,
        },
    )
    theta_log = []
    loss_log = []

    for gen in range(200):  # or however many you want
        thetas = es.ask()
        scores = [evaluate(theta, env) for theta in thetas]

        theta_log.extend(thetas)
        loss_log.extend(scores)

        es.tell(thetas, scores)
        es.disp()

    best_theta = es.result.xbest

    print(best_theta)

    # df_log = pd.DataFrame(theta_log, columns=["Kp", "Ki", "Kd"])
    # df_log["loss"] = loss_log

    # controller = RLCPID([2.12833002, 0.23334387, 0.14534864])
    controller = RLCPID(best_theta)

    sim = RocketSimRL(sim)

    history = [sim.reset(True)]
    done = False

    loss = 0

    while not done:
        action = controller.update(sim.rocket_simulator.get_current_state())
        state, l, done, _ = sim.step(action)
        history.append(state)
        loss += l

    print(loss)
    df = pd.DataFrame(history)
    plot_results(df.to_dict("list"))

    # fig = px.scatter_3d(
    #     df_log, x="Kp", y="Ki", z="Kd", color="loss", size_max=10, opacity=0.7
    # )
    # fig.update_layout(
    #     title="θ-space (Kp, Ki, Kd) vs. Loss",
    #     coloraxis_colorbar=dict(title="Loss"),
    #     template="plotly_dark",
    # )
    # fig.show()
