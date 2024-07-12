import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d
from csv import reader
from pid import PID
from random import random

# PID (From MATLAB Tuner)

Kp = 1.2
Ki = 0.1
Kd = 0.1
pid = PID(Kp, Ki, Kd)
pid.limits = (-5, 5)

pid.setpoint = 0


r = reader(open("f15.csv"))
thrust = [[float(x[0]), float(x[1])] for x in r]
tx = [x[0] for x in thrust]
ty = [x[1] for x in thrust]


def f15_t(t):
    if t < max(tx):
        return interp1d(tx, ty)(t)
    else:
        return 0


def sign(x):
    if x == 0:
        return 1
    else:
        return x / abs(x)


# Engine info
initialEngineMass = .1018
finalEngineMass = .0418
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
totalMass = .670
dryMass = totalMass - 2 * initialEngineMass  # Mass of rocket WITHOUT motors

# Drag
Cd = 1.14
rho = 1.293
A = .00434

# MMOI Calculation
MOMENT_ARM = 0.15
# STRING_LEN = 0.65
# ROTATION_TIME = 1.603
# MOI = totalMass * 9.81 * ROTATION_TIME**2 * MOMENT_ARM**2 / (4 * np.pi**2 * STRING_LEN)
MOI = .0163  # kg m^2

# Timing
t = 0
dt = .01
pid.dt = dt

# Burnouts
stage1_burnout = burnTime
stage2_ignition = 236.1 + 32.98/9.81
stage2_burnout = stage2_ignition + burnTime

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
gimbalAngle = 0
launchAngle = 4

# Angles

theta = launchAngle * np.pi / 180
q = 0
dq = 0

pitch = 90 - launchAngle
if __name__ == "__main__":
    # Main loop
    while True:
        DRAG = .8 * (velY ** 2) * rho * Cd * A * np.sign(velY)

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
            gimbalAngle = 0

        # mass = totalMass
        aa = F / mass

        gimbalAngles.append(gimbalAngle)

        # Torque
        theta += q * dt
        q += dq * dt
        x = np.sin(gimbalAngle * np.pi / 180)
        tau = F * MOMENT_ARM * x
        dq = tau / MOI
        # dq += (random() * 2 - 1)
        theta = theta % (2 * np.pi)

        if theta > np.pi:
            theta -= 2 * np.pi

        thetas.append(theta / np.pi * 180)

        # Newton!
        ax = np.sin(theta) * aa
        ay = np.cos(theta) * aa + g

        ay -= DRAG / mass

        # Integrate a -> vel
        velX += ax * dt
        velY += ay * dt
        vels.append(velY)

        # Integrate vel -> pos
        alt.append(alt[-1] + velY * dt)
        xpos.append(xpos[-1] + velX * dt)

        # Update timestep
        t += dt
        time.append(t)

        # We've burned all our engines and hit the ground
        if t > burnTime and alt[-1] <= 0:
            break

    print("Landing velocity: ", abs(velY))
    print(max(alt))
    print(t)
    print(Kp, Ki, Kd)
    # plt.axvline(x=burnTime, color="orange", linestyle="-")
    # plt.axvline(x=stage2_ignition, color="blue", linestyle="-")
    # plt.axvline(x=stage1_burnout, color="orange", linestyle="-")

    # plt.plot(time, xpos)
    # plt.plot(time, alt)
    plt.plot(time[1:], thetas[1:])
    plt.plot(time[1:], gimbalAngles[1:])

    plt.ylim(-7, 7)
    plt.xlim(0, 10)
    plt.legend(["XPos", "Alt", "Angle", "Gimbal"])

    plt.show()
