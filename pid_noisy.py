from pid import PID

import matplotlib.pyplot as plt
import numpy as np

pid = PID(1.2, .1, 0.1, dt=0.1)
pid.limits = (-12, 12)
pid.Tf = .5
pid.setpoint = 0

bias = 95
l = 100

amp = 2
xs = range(l)
angles = np.random.uniform(-amp, amp, l)

outs = []

for angle in angles:
    outs.append(pid(angle))

plt.plot(xs, outs, angles)
plt.show()
