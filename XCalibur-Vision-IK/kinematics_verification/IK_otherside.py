import roboticstoolbox as rtb
from roboticstoolbox import ET
import numpy as np
from math import pi, degrees
from matplotlib import pyplot as plt
import time

robot = rtb.Robot(
    rtb.ETS([
        ET.tz(qlim=[0, 0.198]),

        ET.tx(0.029),
        ET.Rx(-pi/2),
        ET.tz(0.0125),
        ET.Rz(qlim=[-2*pi, 2*pi]),

        ET.Rx(pi/2),
        ET.tx(0.0275),
        ET.Rz(qlim=[-2*pi, 2*pi]),

        ET.Rx(pi/2),
        ET.tz(0.06823),
        ET.Rz(qlim=[-2*pi, 2*pi]),

        ET.tx(0.02974),
        ET.tz(qlim=[0, 0.095]),
    ]),
    name="XCalibur"
)

robot2 = rtb.Robot(
    rtb.ETS([
        ET.tz(qlim=[0, 0.198]),

        ET.tx(0.029),
        ET.Rx(-pi/2),
        ET.tz(0.0125),
        ET.Rz(qlim=[-2*pi, 2*pi]),

        ET.Rx(pi/2),
        ET.tx(0.0275),
        ET.Rz(qlim=[-2*pi, 2*pi]),

        ET.Rx(pi/2),
        ET.tz(0.06823),
        ET.Rz(qlim=[-2*pi, 2*pi]),

        ET.tx(0.02974),
        ET.tz(qlim=[0, 0.095]),
    ]),
    name="XCalibur"
)

def skew(v):
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def mm_to_m_vec(v):
    result = [0,0,0]
    for i in range(3):
        result[i] = v[i]/1000.0
    return result
 

def ikPt(robot, r, n, q0, 
               max_iter=20, 
               tol=1e-4,
               lam=0.5,
               mu=1e-3):
    
    n = n / np.linalg.norm(n)
    r = mm_to_m_vec(r)
    robot.q = q0
    qd = [0.0, 0.0, 0.0, 0.0, 0.0]
    
    for i in range(max_iter):
        n = n / np.linalg.norm(n)

        # Forward kinematics
        T = robot.fkine(robot.q)
        p = T.t
        R = T.R

        x_axis = R[:, 0]

        # ---- Error vector ----
        pos_error = p - r
        orient_error = np.cross(x_axis, n)

        e = np.hstack((pos_error, orient_error))

        # ---- Jacobian ----
        J = robot.jacob0(robot.q)      # 6 x n
        Jv = J[0:3, :]
        Jw = J[3:6, :]

        # Orientation task Jacobian
        Jo = skew(n) @ skew(x_axis) @ Jw

        J_task = np.vstack((Jv, Jo))

        # ---- Damped Least Squares ----
        JJt = J_task @ J_task.T
        J_pinv = J_task.T @ np.linalg.inv(JJt + mu**2 * np.eye(6))

        # Update
        qd = - lam * J_pinv @ e

        if np.linalg.norm(e) < tol:
            return robot.q

        # Integrate velocity to get new joint positions
        robot.q = robot.q + qd

        # Clamp to joint limits
        for i, link in enumerate(robot.links):
            if link.qlim is not None:
                robot.q[i] = np.clip(robot.q[i], link.qlim[0], link.qlim[1])
        
    print("Did not converge")
    return None

def ik(robot, rArr, nArr, 
               max_iter=20, 
               tol=1e-4,
               lam=0.5,
               mu=1e-3,
               debug=False):
    # default params (before considering other offsets):
    # TODO: integrate these other offsets
    q0 = [0,0,pi/2,pi/2,0]
    result = []

    for i in range(len(rArr)):
        q = ikPt(robot, rArr[i], nArr[i], q0, max_iter, tol, lam, mu)
        if q is None:
            print(i)
            print(rArr[i])
            print(nArr[i])
        q0 = q
        result.append(q)

    return result

'''
r,n
[0.1095, 0.14597, 0.1315]
[ 0.22133524  0.09916774 -0.9701425 ]
we will flip the z
also needs to be in mm
'''

r = [109.5, 145.97, 131.5]
n1 = [0.22133524,  0.09916774, -0.9701425]
n2 = [0.22133524,  0.09916774, 0.9701425]

q0 = [0,0,pi/2,pi/2,0]

q1 = ikPt(robot, r, n1, q0)
q2 = ikPt(robot, r, n2, q0)

print(q1)
print(q2)

'''
[ 0.06320401 -0.42515972  2.67863874  1.79462266  0.08419678]
[0.13948851 0.46254657 2.74215712 1.3130148  0.07342402]
'''

#lets see how these look

import roboticstoolbox as rtb
import swift
from math import pi
from spatialgeometry import Cylinder, Sphere
import spatialmath as sm
from roboticstoolbox import ET
import numpy as np
from spatialgeometry import Mesh
from time import sleep

from verify_kinematics import knife, build_shapes

env = swift.Swift()
env.launch(realtime=True)
env.add(robot)
env.add(robot2)
env.add(knife)

r = mm_to_m_vec(r)

shapes = build_shapes(q1,r,n1,robot)
for s in shapes:
    env.add(s)

shapes = build_shapes(q2,r,n2,robot2)
for s in shapes:
    env.add(s)

env.hold()
