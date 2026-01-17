import numpy as np
import sympy as sp

# initial waypoint (initial waypoint is like the starting position, kinda weird but 0 is for x, 2 is for y, and 4 appears to be for z. Not sure where this initial observation is coming form))
t_steps = 30 # 30 steps for the circle
circle_radius = 1 # m
circle_center = (0, -3, 1) # m
import numpy as np

# adding sampling points info
duration = 15 # s (try making this faster after I'm curious what will happen)
t_scaled = np.linspace(0, t_steps, 10) # t converted to actual time
angle_values = np.linspace(0, 2*np.pi, t_steps)

# Defining the functions of the path
ref_x = np.cos(angle_values) * circle_radius + circle_center[0]
ref_y = np.sin(angle_values) * circle_radius + circle_center[1]
ref_z = np.ones(t_steps) * circle_center[2]

print(ref_x)
print(ref_y)
print(ref_z)