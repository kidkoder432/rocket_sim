import openpyxl
from math import cos, exp, pi, sqrt
from numpy import random
from pid import PID
import matplotlib.pyplot as plt

eo, oo = [], []

L = 2
g = -9.81
d2r = lambda x: x * pi / 180
r2d = lambda x: x * 180 / pi

b = 0.05453874
m = 0.67

# Noiser function
randomizer = lambda x, h: h(x) + random.normal(-.5, .5)


# Pendulum motion function
pendulum = lambda t: r2d(d2r(5) * cos(sqrt(-g / L) * t) * exp(b * t / 2 * m))
pendulum_noise = lambda x: randomizer(x, pendulum)


def step_ramp(t):
    if t > 2:
        return 4 - t / 5
    else:
        return 2


step_noise = lambda x: randomizer(x, step_ramp)

wb = openpyxl.load_workbook("rocket_data.xlsx", read_only=True)

print(wb.sheetnames)
sheet = wb["Sheet2"]


def scale(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


allErors = 0

pid = PID(Kp=1.2, Ki=0.1, Kd=0.1, N=5, dt=0.001)
pid.limits = (-5, 5)
pid.setpoint = 0

for t in range(10000):
    t = t / 1000
    xin = pendulum_noise(t)

    expected_xout = pid(xin, 0.001)
    xout = 95
    xout = scale(xout, 83, 107, -5, 5)

    allErors += abs(xout - expected_xout)
    if t * 100 % 100 == 0:
        print(
            f"X: Input: {xin}, Expected Output: {expected_xout}, Output (scaled): {xout}, Error: {xout - expected_xout}, Time: {t}"
        )
    lastTime = t

    eo.append(expected_xout)
    oo.append(xin)

print("Average error: ", allErors / 10000)

xs = range(len(eo))
plt.plot(xs, eo, oo)
plt.legend(["Expected", "Input"])

plt.show()
