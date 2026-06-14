import numpy as np
import matplotlib.pyplot as plt

g = 9.81
P = 200
m = 0.454
b = 0.026  
W = 0.40 
R_target = 20  
r = 3

def release_speed(r, n, P, m):
    return 2 * (np.pi * r * n * P / m)**(1/3)


def range_no_drag(v, gamma_deg):
    gamma = np.radians(gamma_deg)
    return v**2 * np.sin(2 * gamma) / g


def simulate_projectile_drag(v0, gamma_deg, m, b, Cd=0.4, rho=1.225, dt=0.001):
    gamma = np.radians(gamma_deg)

    A = np.pi * b**2

    x, y = 0.0, 1.6
    vx = v0 * np.cos(gamma)
    vy = v0 * np.sin(gamma)

    xs, ys = [x], [y]

    while y >= 0:
        v = np.sqrt(vx**2 + vy**2)

        Fd = 0.5 * rho * Cd * A * v**2

        ax = -Fd * vx / (m * v)
        ay = -g - Fd * vy / (m * v)

        vx += ax * dt
        vy += ay * dt

        x += vx * dt
        y += vy * dt

        xs.append(x)
        ys.append(y)

    return np.array(xs), np.array(ys)


def spin_rate(v, r, b):
    return v / np.sqrt(2 * np.pi * r * b)


def timing_error(W, r, R, gamma_deg):
    gamma = np.radians(gamma_deg)
    return W * r * np.sqrt(np.sin(2 * gamma) / (4 * g * R**3))

