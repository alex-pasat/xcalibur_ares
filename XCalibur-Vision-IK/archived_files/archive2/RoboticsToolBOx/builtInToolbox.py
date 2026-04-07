import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
from math import pi

def pose_from_normal(r, n):
    # Make unit vector
    n = n / np.linalg.norm(n)
    
    # Build orthonormal frame
    if abs(n[0]) < 0.9:
        v = np.array([1,0,0])
    else:
        v = np.array([0,1,0])
    
    z = n
    y = np.cross(z, v)
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    
    R = np.column_stack([x, y, z])
    T = SE3(r) * SE3.Rt(R, np.zeros(3))
    return T

def ik_tangent_rtb(robot, r, n, q0):
    T_des = pose_from_normal(r, n)
    
    sol = robot.ikine_LM(
        T_des,
        q0=q0,
        mask=[1, 1, 1, 0, 1, 1]  # free rotation about x-axis
    )
    
    if not sol.success:
        raise RuntimeError("IK did not converge")
    
    return sol.q

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
q0 = [0,0,pi/2,pi/2,0]

r = np.array([180, -20, 60])
n = np.array([0, 0, 1])

q_sol = ik_tangent_rtb(robot, r, n, q0)
print(robot.fkine(q_sol))
