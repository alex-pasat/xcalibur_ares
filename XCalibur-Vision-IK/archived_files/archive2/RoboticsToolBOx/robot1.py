import roboticstoolbox as rtb
import numpy as np
from spatialmath import SE3
from math import pi

# Define links using standard DH parameters
L1 = rtb.RevoluteDH(a=1, alpha=0, d=0)
L2 = rtb.RevoluteDH(a=1, alpha=0, d=0)

# Create robot
robot = rtb.DHRobot([L1, L2], name="Planar2R")

print(robot)

q = [pi/4, pi/4]   # joint angles

T = robot.fkine(q)
print(T)

# robot.plot([pi/4, pi/4])
# input("Press Enter to close...")

T_target = SE3(1.5, 0.5, 0)

sol = robot.ikine_LM(T_target)
print(sol.q)

robot.plot(sol.q)
input("Press Enter to close...")

# This looks good!
# Just apply to our robot!

L1 = rtb.PrismaticDH(
    theta=0,   # fixed
    a=0,
    alpha=0    # no twist
)

robot = rtb.DHRobot([L1], name="Slider")
print(robot)

robot.plot([1])
input("Press Enter to close...")