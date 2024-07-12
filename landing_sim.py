from csv import reader

import numpy as np
import plotly.express as px
from scipy.interpolate import interp1d

MASS = .602
BURN_TIME = 3.45
RETRO_TIME = 3

# APOGEE = (.5 * (THRUST / MASS - 9.81) * BURN_TIME ** 2
#           + 1 / 2 * 9.81 * ((THRUST / MASS - 9.81) * BURN_TIME / 9.81) ** 2)

Cd = 1.14
rho = 1.293
A = .00434

dt = 0.01

initialEngineMass = .103
finalEngineMass = .043

r = reader(open("f15.csv"))
thrust = [[float(x[0]), float(x[1])] for x in r]
tx = [x[0] for x in thrust]
ty = [x[1] for x in thrust]
lerp = interp1d(tx, ty)


def engine_thrust(t):
    if t < max(tx):
        return lerp(t)
    else:
        return 0


def engine_mass(t):  # Return lerped mass of engine
    if t > BURN_TIME:
        return finalEngineMass
    elif t < 0:
        return initialEngineMass
    else:
        return ((finalEngineMass - initialEngineMass) * t + BURN_TIME * initialEngineMass) / BURN_TIME


def sim(burnAlt, m, all=False):
    global MASS, BURN_TIME, A
    ay = -9.80665
    thrust = 0
    vy = 0
    y = .1
    time = 0
    t = T = 0
    mass = m
    dryMass = m - .206

    ys = []
    ts = []
    vs = []
    ms = []
    apg = -10
    apgTime = 0
    invalid = False
    apgHit = True
    while True:

        DRAG = .8 * (vy ** 2) * rho * Cd * A * np.sign(vy)
        ts.append(time)

        if 0 < time < BURN_TIME:
            thrust = engine_thrust(time)
            mass = dryMass + initialEngineMass + engine_mass(time)
        elif BURN_TIME <= time:
            thrust = 0
            mass = dryMass + initialEngineMass + finalEngineMass

        if 0 < t - T < BURN_TIME:
            thrust = engine_thrust(t - T)
            mass = dryMass + engine_mass(t - T)
        elif t - T > BURN_TIME:
            thrust = 0
            mass = dryMass + finalEngineMass

        ms.append(ay)

        if y > apg:
            apg = y

            T = np.sqrt(2 * abs(apg - burnAlt) / 9.80665)
            if time < BURN_TIME:
                ay = thrust / mass
            else:
                ay = 0

        elif y <= apg and time > BURN_TIME:
            if apg < burnAlt:
                invalid = True
            if apgHit:
                apgTime = time - .02
                apgHit = False
            apg = 100000
            t = time - apgTime
            if t < T or t > T + BURN_TIME:
                ay = 0
            else:
                ay = thrust / mass
        else:
            ay = thrust / mass

        ay -= 9.80665
        ay -= DRAG / mass
        vy += ay * dt
        y += vy * dt

        ys.append(y)
        vs.append(vy)
        time += dt
        # print(apg, apgTime, T)
        # if 0 < t - T < BURN_TIME:
        #     if y < 0:
        #         if vy < -20 or 0 < t - T < 1:
        #             invalid = True
        #             break
        #         continue
        if y < 0:
            if time < 5:
                continue
            if t - T < 3:
                invalid = True
            break

    if vy > 0:
        invalid = True
    if not all:
        if invalid:
            return 0
        else:
            return vs[-1]
    else:
        return ts, ys, vs, ms, apgTime + T


if __name__ == "__main__":
    m = float(input("Enter mass of rocket: "))
    t = float(input("Enter burn start altitude: "))
    ts, ys, vs, ms, at = sim(t, m, True)
    print(vs[-1])
    fig = px.line(x=ts, y=ys)
    fig.add_scatter(x=ts, y=vs)
    fig.add_scatter(x=ts, y=ms)
    fig.add_vline(x=at, line_dash='dash')
    fig.show()

    exit()
