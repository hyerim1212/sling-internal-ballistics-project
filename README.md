# Physics Project: Step-by-Step Mathematical Derivations and Python 
202611202, Hyerim Jung, General physics I, section 02, New Group 12

## sling-internal-ballistics
I wrote Python codes for calculation and simulation to visualize the effect of air drag on the trajectory of a sling bullet.

# First python file - compare_drag
This file compares the projectile motion of the sling bullet with and without air drag.

- It generates a graph showing both the no-drag trajectory and the trajectory with air drag.
- Users can substitute their own values for parameters such as the launch angle gamma, sling radius r, and number of rotations n.
- Based on these input values, the code calculates and visualizes the range of the bullet under both no-drag and drag conditions.
- The equations implemented in this code are based on the simplified model presented in the original paper.
Main equations implemented:

Release speed `v = 2 * (np.pi * r * n * P / m)**(1/3)`

No drag Range = `v **2 * np.sin(2 * gamma) / g`


# Second python file - vpython_sling_3d
- The simulation follows the paper's simplified pirouette-style sling model.
  
  The bullet moves in a circular arc in one fixed plane.

  The slinger applies constant angular acceleration during wind-up.

  The sling cords are stiff and massless.

  Gravity and drag are ignored during internal ballistics.

  Quadratic air drag is included after release for the external trajectory.


- Main equations implemented:
  
  Release speed: `v = 2 * (pi * r * n * P / m)^(1/3)`

  Spin rate: `f = v / sqrt(2 * pi * r * b)`
  
  Timing error: `dt = W * r * sqrt(sin(2 gamma) / (4 * g * R^3))`
  
  Default parameters match the paper's example: `P = 200 W`, `r = 2.5 m`,
  `m = 0.454 kg`, and `n = 3`.

  This code runs with VPython. If vpython is not installed, please install vpython.
## extra files - packages and modules
- set_of_equation/
  equation.py : Physics model used in compare_drag.py
  
- sling_model/
  sling_model.py : Physics model used in vpython_sling_3d.py
