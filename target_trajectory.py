import math

import numpy as np

# get trajectory point for some time in seconds
def circle_traj_point(t):
    return point_on_circle(t*360)

# get point on circle shape
# for 0 deg point is at the top of circle and 180 deg at the bottom
def point_on_circle(deg=0, center_x=-0.3, center_z=0.5, r=0.1):
    deg = deg % 360
    angle = math.pi * deg / 180
    x = center_x + (r * math.sin(angle))
    z = center_z + (r * math.cos(angle))
    return np.array([x, 0, z])
