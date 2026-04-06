import roboticstoolbox as rtb
import numpy as np
from spatialgeometry import Cylinder, Box, Sphere
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R
from math import pi, degrees

def skew(v):
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def ik_tangent(robot, r, n, q0, 
               max_iter=200, 
               tol=1e-12,
               lam=0.5,
               mu=1e-3):
    
    n = n / np.linalg.norm(n)
    q = q0.copy()
    
    for i in range(max_iter):
        
        # Forward kinematics
        T = robot.fkine(q)
        p = T.t
        R = T.R
        
        x_axis = R[:, 0]
        
        # ---- Error vector ----
        pos_error = p - r
        orient_error = np.cross(x_axis, n)
        
        e = np.hstack((pos_error, orient_error))
        
        if np.linalg.norm(e) < tol:
            print("Converged in", i, "iterations")
            return q
        
        # ---- Jacobian ----
        J = robot.jacob0(q)      # 6 x n
        Jv = J[0:3, :]
        Jw = J[3:6, :]
        
        # Orientation task Jacobian
        Jo = skew(n) @ skew(x_axis) @ Jw
        
        J_task = np.vstack((Jv, Jo))
        
        # ---- Damped Least Squares ----
        JJt = J_task @ J_task.T
        J_pinv = J_task.T @ np.linalg.inv(JJt + mu**2 * np.eye(6))
        
        # Update
        q = q - lam * J_pinv @ e
        
    print("Did not converge")
    return q

# Define the links
# First, the underpass
L1 = rtb.PrismaticMDH(theta=0, a=0, alpha=0, qlim=[0, 198])
L2 = rtb.RevoluteMDH(a=29, alpha=-pi/2, d=-12.5, qlim=[0, 2*pi])
L3 = rtb.RevoluteMDH(a=27.5, alpha=pi/2, d=-0, qlim=[0, 2*pi])
L4 = rtb.RevoluteMDH(a=0, alpha=pi/2, d=68.23, qlim=[0, 2*pi])
L5 = rtb.PrismaticMDH(theta=0, a=29.74, alpha=0, qlim=[0, 95])

# Create robot
robot = rtb.DHRobot([L1, L2, L3, L4, L5], name="Robot")
print(robot)

# default params:
# [0,0,pi/2,pi/2,0]
q0 = [0,0,pi/2,pi/2,0]

# r = np.array([180, -20, 60])
r = np.array([137.26,58.11,178.98])
n = np.array([0.26, -0.01, -0.97])
# rot = R.from_euler('ZY', [-90.48, 15], degrees=True)  # Z then Y rotation
# R_mat = rot.as_matrix()  # 3x3 rotation matrix
# n = R_mat @ n
print(n)

q_sol = ik_tangent(robot, r, n, q0)

def degQ(q):
    r = q.copy()
    for i in range(1,4):
        r[i] = degrees(r[i])
    return r

print("target r = ", r)
print("n = ", n)
print("q sol = ", degQ(q_sol))
print(robot.fkine(q_sol))

robot.plot(q_sol)
input("Press Enter to close...")
