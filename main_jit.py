import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline
from csv import reader
from pid import PID
from numba import jit

r = reader(open("f15.csv"))
thrust = [[float(x[0]), float(x[1])] for x in r]
tx = [x[0] for x in thrust]
ty = [x[1] for x in thrust]


def f15_t(t):
    if t < max(tx):
        return CubicSpline(tx, ty)(t)
    else:
        return 0.0


def sign(x):
    if x == 0.0:
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
    elif t < 0.0:
        return initialEngineMass
    else:
        return ((finalEngineMass - initialEngineMass) * t + burnTime * initialEngineMass) / burnTime


@jit
def main():
    # PID
    Kp = 1
    Ki = 1
    Kd = 0.0
    pid = PID(Kp, Ki, Kd)
    pid.limits = (-5, 5)

    pid.setpoint = 0.0

    # Accel + vel
    ax = 0.0
    ay = g = -9.81
    velX = 0.0
    velY = 0.0

    # Mass
    totalMass = .868
    dryMass = totalMass - 2 * initialEngineMass  # Mass of rocket WITHOUT motors

    MOI = .05  # TODO: Replace with Actual MMOI

    # Timing
    t = 0.0
    dt = .01
    pid.dt = dt

    # Burnouts
    stage1_burnout = 3.45
    stage2_ignition = 9
    stage2_burnout = stage2_ignition + burnTime

    # Plot arrays
    alt = [0.0]
    xpos = [0.0]
    time = [0.0]
    vels = [0.0]
    pitches = [0.0]
    thetas = [0.0]
    gimbalAngles = [0.0]

    masses = [dryMass + 2 * initialEngineMass]

    # Gimbaling
    gimbalAngle = 0.0
    launchAngle = 4

    # Angles

    theta = launchAngle * np.pi / 180.0
    q = 0.0

    R = .3

    while True:

        if 0.0 <= t <= stage1_burnout:
            F = f15_t(t)
            mass = dryMass + initialEngineMass  # Mass of unlit 2nd stage engine
            mass += f15_m(t)  # Mass of burning 1st stage engine

        elif stage1_burnout <= t <= stage2_ignition:
            F = 0.0
            mass = dryMass + initialEngineMass  # Mass of unlit 2nd stage engine

        elif stage2_ignition <= t <= stage2_burnout:
            F = f15_t(t - stage2_ignition)
            mass = dryMass + f15_m(t - stage2_ignition)  # Mass of burning 2nd stage engine
        else:
            mass = dryMass + 2 * finalEngineMass

        aa = F / mass

        gimbalAngle = pid(theta * 180.0 / np.pi)
        print(theta / np.pi * 180.0, pid.components)

        gimbalAngles.append(gimbalAngle)
        # print(gimbalAngle)
        # Torque
        x = np.sin(gimbalAngle * np.pi / 180.0)
        tau = F * R * x
        dq = tau / MOI
        q += dq * dt
        theta += q * dt

        theta = theta % (2 * np.pi)

        if theta > np.pi:
            theta -= 2 * np.pi

        thetas.append(theta / np.pi * 180.0)

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
        if t > burnTime and alt[-1] <= 0.0:
            break
    return velY, alt, t, time, gimbalAngles


velY, alt, t, time, gimbalAngles = main()
print("Landing velocity: ", abs(velY))
print(max(alt))
print(t)

# plt.axvline(x=burnTime, color="orange", linestyle="-")
# plt.axvline(x=stage2_ignition, color="blue", linestyle="-")

# plt.plot(time, alt)
plt.plot(time[1:], gimbalAngles[1:])
plt.show()
