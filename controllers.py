import numpy as np
from typing import Dict, Any

GIMBAL_DEG = 20

class BangBangController(object):
    def __init__(self, theta):
        self.theta = theta

    def update(self, state):

        neg_thresh = self.theta[0]
        pos_thresh = self.theta[1]

        th = state["theta_measured_radians"] * 180 / np.pi
        if th < neg_thresh:
            return GIMBAL_DEG
        elif th > pos_thresh:
            return -GIMBAL_DEG
        else:
            return 0

    def reset(self):
        pass


class DummyController(object):

    def __init__(self, control):
        
        self.control = control

    def update(self, state: Dict[str, Any]) -> float:
        
        return self.control

    def reset(self, control: float=None):
        if control is not None:
            self.control = control
