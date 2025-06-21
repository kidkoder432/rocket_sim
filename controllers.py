import numpy as np

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