from csv import reader

import matplotlib.pyplot as plt
import numpy as np
from random import random
from scipy.interpolate import CubicSpline

from pid import PID

# PID (From MATLAB Tuner)

Kp = 6.58679935818825
Ki = 18.6642739815057
Kd = 0.482664861486399
pid = PID(Kp, Ki, Kd)
pid.limits = (-5, 5)

pid.setpoint = 0


r = reader(open("f15.csv"))
thrust = [[float(x[0]), float(x[1])] for x in r]
tx = [x[0] for x in thrust]
ty = [x[1] for x in thrust]


def f15_t(t):
    if t < max(tx):
        return CubicSpline(tx, ty)(t)
    else:
        return 0


def sign(x):
    if x == 0:
        return 1
    else:
        return x / abs(x)


# Engine info
initialEngineMass = .094
finalEngineMass = .0338
burnTime = 3.45


def f15_m(t):  # Return lerped mass of engine
    if t > burnTime:
        return finalEngineMass
    elif t < 0:
        return initialEngineMass
    else:
        return ((finalEngineMass - initialEngineMass) * t + burnTime * initialEngineMass) / burnTime


# Accel + vel
ax = 0
ay = g = -9.81
velX = 0
velY = 0

# Mass
totalMass = .868
dryMass = totalMass - 2 * initialEngineMass  # Mass of rocket WITHOUT motors

# MMOI Calculation
MOMENT_ARM = 0.3
STRING_LEN = 0.65
ROTATION_TIME = 1.603
# MOI = totalMass * 9.81 * ROTATION_TIME**2 * MOMENT_ARM**2 / (4 * np.pi**2 * STRING_LEN)

MOI = .11

# Timing
t = 0
dt = .02
pid.dt = dt

# Burnouts
stage1_burnout = burnTime
stage2_ignition = 23

# Plot arrays
alt = [0]
xpos = [0]
time = [0]
vels = [0]
pitches = [0]
thetas = [0]
gimbalAngles = [0]

masses = [dryMass + 2 * initialEngineMass]

# Gimbaling
launchAngle = 0

# Angles
q = 0
R = .3


def sim(altCoef):
    MOI_S = .11
    theta = launchAngle * np.pi / 180

    # altCoef is a number in [0,1] 
    # Main loop
    t = 0
    q = 0

    # Plot arrays
    global alt, xpos, thetas, time, gimbalAngles, masses, velY, velX, stage2_ignition

    velY = 0
    velX = 0

    masses = [dryMass + 2 * initialEngineMass]

    gimbalAngle = 0

    # Plot arrays
    alt = [0]
    xpos = [0]
    time = [0]
    thetas = [0]
    gimbalAngles = [0]

    stage2_ignition = 3.45 + altCoef
    stage2_burnout = stage2_ignition + burnTime

    while True:

        gimbalAngle = pid(theta * 180/np.pi)

        if 0 <= t <= stage1_burnout:
            F = f15_t(t)
            mass = dryMass + initialEngineMass  # Mass of unlit 2nd stage engine
            mass += f15_m(t)  # Mass of burning 1st stage engine

        elif stage1_burnout <= t <= stage2_ignition:
            F = 0
            gimbalAngle = 0
            mass = dryMass + initialEngineMass  # Mass of unlit 2nd stage engine

        elif stage2_ignition <= t <= stage2_burnout:
            F = f15_t(t - stage2_ignition)
            mass = dryMass + f15_m(t - stage2_ignition)  # Mass of burning 2nd stage engine
        else:
            mass = dryMass + 2 * finalEngineMass

        aa = F / mass
        masses.append(mass)
        # print(theta / np.pi * 180, pid.components)

        gimbalAngles.append(gimbalAngle)
        # print(gimbalAngle)

        # Torque
        MOI = MOI_S * (mass / totalMass)
        x = np.sin(gimbalAngle * np.pi / 180)
        tau = F * R * x
        # tau += (random() * 6 - 3) * .05
        dq = tau / MOI
        q += dq * dt
        theta += q * dt

        theta = theta % (2 * np.pi)

        if theta > np.pi:
            theta -= 2 * np.pi

        thetas.append(theta / np.pi * 180)

        # Newton!
        ax = np.sin(theta) * aa
        ay = np.cos(theta) * aa + g

        # Integrate a -> vel
        velX += ax * dt
        velY += ay * dt

        # Integrate vel -> pos
        alt.append(alt[-1] + velY * dt)
        xpos.append(xpos[-1] + velX * dt)

        # Update timestep
        t += dt
        time.append(t)

        # We've burned all our engines and hit the ground
        if t > burnTime and alt[-1] <= 0:
            break

    return velY


times = np.arange(4, 6, 0.02)
lvels = []

for x in times:
    lvels.append(sim(x))
    print(x)

best = float(times[lvels.index(max(lvels))])

print(best)
sim(best)

# plt.axvline(x=burnTime, color="orange", linestyle="-")
# plt.axvline(x=stage2_ignition, color="blue", linestyle="-")

plt.axvline(best, color="red")

plt.scatter(times, lvels)


plt.plot(time, alt, color="red")
plt.plot(time[1:], masses[1:], color='black')
plt.plot(time[1:], gimbalAngles[1:])

# plt.ylim(-7, 7)

plt.show()
