import roboticstoolbox as rtb
import numpy as np
from spatialgeometry import Cylinder, Box, Sphere
from spatialmath import SE3
from math import pi
from scipy.optimize import minimize

# Define the links
# First, the underpass
L1 = rtb.PrismaticMDH(theta=0, a=0, alpha=0, qlim=[0, 198])
L2 = rtb.RevoluteMDH(a=29, alpha=-pi/2, d=-12.5, qlim=[0, 2*pi])
L3 = rtb.RevoluteMDH(a=26.5, alpha=pi/2, d=-0, qlim=[0, 2*pi])
L4 = rtb.RevoluteMDH(a=0, alpha=pi/2, d=68.23, qlim=[0, 2*pi])
L5 = rtb.PrismaticMDH(theta=0, a=29.74, alpha=0, qlim=[0, 95])

# Create robot
robot = rtb.DHRobot([L1, L2, L3, L4, L5], name="Robot")
print(robot)

# default params:
# [0,0,pi/2,pi/2,0]

# plot it
# robot.plot([20,0,pi/2,pi/2,40])

# IK
# Desired end-effector position
p_des = np.array([180, -20, 60])
# Desired x-axis direction of end-effector
x_des = np.array([1, 0, 0])

# Cost function: position error + x-axis orientation error
def cost(q):
    T = robot.fkine(q)
    pos_err = np.linalg.norm(T.t - p_des)
    x_axis_err = np.linalg.norm(T.R[:,0] - x_des)
    return pos_err + x_axis_err

# Initial guess
q0 = np.zeros(5)

# Solve using numerical optimization
res = minimize(cost, q0, bounds=[L1.qlim, L2.qlim, L3.qlim, L4.qlim, L5.qlim])
sol = res.x
print("error = ", cost(sol))
print(sol)
print(robot.fkine(sol))
# robot.plot(sol)
input("Press Enter to close...")
