import numpy as np
import matplotlib.pyplot as plt

dt = 1
accel_data = [0] * 1000 + np.random.normal(0, 0.1, 1000)
gyro_data = [0] * 1000 + np.random.normal(0, 0.1, 1000)

gyro_integrated = []
t = 0
for i in gyro_data:
    t += i
    gyro_integrated.append(t * dt)

del t

rotation_filtered = []
r = 0
TAU = 0.97
for i, g in enumerate(gyro_data):
    accel = accel_data[i]
    gyro = g
    r += gyro * dt
    r *= TAU
    r += accel * (1 - TAU)
    rotation_filtered.append(r)

plt.plot(accel_data, label='accel')
plt.plot(gyro_data, label='gyro')
plt.plot(gyro_integrated, label='integ')
plt.plot(rotation_filtered, label='filter')

plt.ylim(min(gyro_integrated), max(gyro_integrated))

plt.show()
