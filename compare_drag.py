import numpy as np
import matplotlib.pyplot as plt
from set_of_equation.equation import*

gamma = float(input("Enter a Degree γ :"))
r = float(input("Enter a Radius r(m):"))      
n = int(input("Enter a Rotation Number n:"))

speeds = np.linspace(5, 80, 50)

ranges_no_drag = []
ranges_drag = []

for speed in speeds:
    x = range_no_drag(speed, gamma)
    ranges_no_drag.append(x)

    xs, ys = simulate_projectile_drag(speed, gamma, m, b)
    ranges_drag.append(xs[-1])

plt.figure(figsize=(7, 5))
plt.plot(speeds, ranges_no_drag, label="No drag")
plt.plot(speeds, ranges_drag, label="With drag")
plt.xlabel("Release speed v (m/s)")
plt.ylabel("Range R (m)")
plt.title("Range - release speed")
plt.grid(True)
plt.legend()
plt.show()
