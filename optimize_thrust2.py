import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline
from csv import reader
from pprint import pprint

# Load thrust curve data
r = reader(open("f15.csv"))
thrust = [[float(x[0]), float(x[1])] for x in r]
tx = [x[0] for x in thrust]
ty = [x[1] for x in thrust]


def f15_t(t):  # Return cubic-splined thrust
    if 0 < t < max(tx):
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
    if t > stage1_burnout:
        return finalEngineMass
    elif t < 0:
        return initialEngineMass
    else:
        return ((finalEngineMass - initialEngineMass) * t + burnTime * initialEngineMass) / burnTime


stage1_burnout = 3.45  # Stage 1 burnout time

landingVel = -1000
bestLandingVel = -2 ** 64

alts = []
times = []
lastError = -100

g = -9.81  # Gravitational constant

totalMass = .868

dryMass = totalMass - 2 * initialEngineMass  # Mass of rocket WITHOUT motors
dt = .01  # TIme step

stage2_ignition = 30
while abs(lastError) > .1:

    alt = [0]
    time = [0]
    vels = [0]
    a = g
    masses = [dryMass + 2 * initialEngineMass]
    vel = 0
    wetMass = dryMass + f15_m(0)

    stage2_burnout = stage2_ignition + 3.45

    t = 0

    while True:

        if 0 <= t <= stage1_burnout:
            F = f15_t(t)
            mass = dryMass + initialEngineMass  # Mass of unlit 2nd stage engine
            mass += f15_m(t)  # Mass of burning 1st stage engine

        elif stage1_burnout <= t <= stage2_ignition:
            F = 0
            mass = dryMass + initialEngineMass + finalEngineMass  # Mass of unlit 2nd stage engine

        elif stage2_ignition <= t <= stage2_burnout:
            F = f15_t(t - stage2_ignition)
            mass = dryMass + finalEngineMass + f15_m(t - stage2_ignition)  # Mass of burning 2nd stage engine
        else:
            mass = dryMass + 2 * finalEngineMass
        masses.append(mass * 10)

        # Discretely integrate to get vel and alt
        alt.append(alt[-1] + vel * dt)  # Integrate vel -> alt; clamp alt to 0 because the ground is solid

        vel += a * dt  # Integrate a -> vel
        vels.append(vel)

        a = F / mass + g  # Newton!

        # Update timestep
        t += dt
        time.append(t)

        # We've burned all our engines and hit the ground
        if alt[-1] <= 0 and t > stage2_ignition:
            break

    # Optimization of landing velocity w.r.t landing motor start time
    landingVel = vel
    if landingVel > bestLandingVel:
        bestLandingVel = landingVel
        alts.append(alt)
        times.append(time)
    stage2_ignition += (landingVel + .5) / 100
    print("Motor ignition: ", stage2_ignition)
    print("Landing vel:", landingVel)
    lastError = landingVel + .5
    print("Error: ", landingVel + .5)

print("Optimum landing time (after launch):", stage2_ignition, "s")
print("Apogee:", max(alts[-1]), "m")
print("Landing vel:", landingVel, 'm/s')

plt.axvline(x=burnTime, color="orange", linestyle="--")
plt.axvline(x=stage2_ignition, color="blue", linestyle="--")

# for i in range(len(times) - 1):
#     plt.plot(times[i], alts[i], color="g")

plt.plot(times[-1], alts[-1], color="r")
plt.plot(times[-1], vels, color="darkblue")

plt.show()
