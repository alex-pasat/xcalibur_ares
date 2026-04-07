import roboticstoolbox as rtb
from roboticstoolbox import ET
import numpy as np
from math import pi, degrees
from matplotlib import pyplot as plt
import time

robot = rtb.Robot(
    rtb.ETS([
        ET.tz(qlim=[0, 0.265]),

        ET.tx(0.029),
        ET.Rx(-pi/2),
        ET.tz(0.0125),
        ET.Rz(qlim=[-pi, pi]),

        ET.Rx(pi/2),
        ET.tx(0.0275),
        ET.Rz(qlim=[-pi, pi]),

        ET.Rx(pi/2),
        ET.tz(0.06823),
        ET.Rz(qlim=[-pi, pi]),

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
               max_iter=100, 
               tol=1e-4,
               lam=1,
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

def ik(robot, rArr, nArr, q0,
               max_iter=100, 
               tol=1e-4,
               lam=0.5,
               mu=1e-3,
               debug=False):
    # default params (before considering other offsets):
    # TODO: integrate these other offsets
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
