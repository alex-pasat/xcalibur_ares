import roboticstoolbox as rtb
import swift
from math import pi, sin, radians, degrees
from spatialgeometry import Cylinder, Sphere
import spatialmath as sm
from roboticstoolbox import ET
import numpy as np
from spatialgeometry import Mesh
from time import sleep

RADIUS = 0.004  # 4mm in metres

# ─── Robot Definition ────────────────────────────────────────────────────────
robot = rtb.Robot(
    rtb.ETS([
        ET.tz(qlim=[0, 0.265]),

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

ORANGE = (1.0, 0.4, 0.1, 1.0)
BLUE = (0.2, 0.6, 1.0, 1.0)
RED = (1.0, 0.0, 0.0, 1.0)

# ─── Geometry builders ───────────────────────────────────────────────────────
def cyl_x(length, frame=sm.SE3(), color=BLUE):
    pose = frame * sm.SE3.Tx(length / 2) * sm.SE3.Ry(pi / 2)
    return Cylinder(radius=RADIUS, length=abs(length), pose=pose.A, color=color)

def cyl_z(length, frame=sm.SE3(), color=BLUE):
    pose = frame * sm.SE3.Tz(length / 2)
    return Cylinder(radius=RADIUS, length=abs(length), pose=pose.A, color=color)

def sphere(frame=sm.SE3(), color=ORANGE):
    return Sphere(radius=RADIUS * 1.5, pose=frame.A, color=color)

roll_mechanism = Mesh(
    filename=r"C:\Vision\CAD\ROLL_MECHANISM.STL",
    scale=(0.001, 0.001, 0.001),
    color=(0.2, 0.6, 1.0, 1.0)
)

def make_normal_arrow(r, n, length=0.05, color=(1.0, 0.0, 0.0, 1.0)):
    """Cylinder from r in direction n."""
    n = np.array(n) / np.linalg.norm(n)
    center = np.array(r) + n * length / 2

    # Find rotation from Z axis to n
    z = np.array([0, 0, 1])
    axis = np.cross(z, n)
    if np.linalg.norm(axis) < 1e-6:
        # n is parallel to z
        T = sm.SE3.Tx(center[0]) * sm.SE3.Ty(center[1]) * sm.SE3.Tz(center[2])
    else:
        angle = np.arccos(np.clip(np.dot(z, n), -1, 1))
        axis = axis / np.linalg.norm(axis)
        T = sm.SE3(center) * sm.SE3.AngleAxis(angle, axis)

    return Cylinder(radius=RADIUS, length=length, pose=T.A, color=color)

def build_shapes(q, r, n):
    O = sm.SE3()

    T0 = sm.SE3(robot.fkine(q, end=robot.links[0]).A)
    T0_tx = T0 * sm.SE3.Tx(0.029)
    T0_rx = T0_tx * sm.SE3.Rx(-pi/2)

    T1 = sm.SE3(robot.fkine(q, end=robot.links[1]).A)
    T1_rx = T1 * sm.SE3.Rx(pi/2)

    T2 = sm.SE3(robot.fkine(q, end=robot.links[2]).A)
    T2_rx = T2 * sm.SE3.Rx(pi/2)
    T2_tz = T2_rx * sm.SE3.Tz(0.06823)

    T3 = sm.SE3(robot.fkine(q, end=robot.links[3]).A)
    T3_tx = T3 * sm.SE3.Tx(0.02974)

    T3_roll = T3 * sm.SE3.Rz(-pi/2) * sm.SE3.Ty(-0.02) * sm.SE3.Tx(-0.029) * sm.SE3.Tz(-0.008)

    T4 = sm.SE3(robot.fkine(q, end=robot.links[4]).A)

    # target position
    Tr = O * sm.SE3.Tx(r[0]) * sm.SE3.Ty(r[1]) * sm.SE3.Tz(r[2])
    

    return [
        sphere(O),

        cyl_z(q[0], O),
        sphere(T0),

        cyl_x(0.029, T0),
        sphere(T0_tx,color=BLUE),
        cyl_z(0.0125, T0_rx),
        sphere(T1),

        cyl_x(0.029, T1_rx),
        sphere(T2),

        cyl_z(0.06823, T2_rx),
        sphere(T3),

        Mesh(
            filename=r"C:\Vision\CAD\ROLL_MECHANISM.STL",
            scale=(0.001, 0.001, 0.001),
            pose=(T3_roll).A,
            color=(0.2, 0.6, 1.0, 1.0)
        ),

        cyl_x(0.02974, T3),
        sphere(T3_tx,color=BLUE),
        cyl_z(q[4], T3_tx),
        sphere(T4),

        sphere(Tr, color=RED),
        make_normal_arrow(r, n, length=0.05, color=RED),
    ]

# ─── Update shapes ───────────────────────────────────────────────────────────
def update_shapes(shapes, q):
    O = sm.SE3()

    T0 = sm.SE3(robot.fkine(q, end=robot.links[0]).A)
    T0_tx = T0 * sm.SE3.Tx(0.029)
    T0_rx = T0_tx * sm.SE3.Rx(-pi/2)

    T1 = sm.SE3(robot.fkine(q, end=robot.links[1]).A)
    T1_rx = T1 * sm.SE3.Rx(pi/2)

    T2 = sm.SE3(robot.fkine(q, end=robot.links[2]).A)
    T2_rx = T2 * sm.SE3.Rx(pi/2)

    T3 = sm.SE3(robot.fkine(q, end=robot.links[3]).A)
    T3_tx = T3 * sm.SE3.Tx(0.02974)
    T3_roll = T3 * sm.SE3.Rz(-pi/2) * sm.SE3.Ty(-0.02) * sm.SE3.Tx(-0.029) * sm.SE3.Tz(-0.008)

    T4 = sm.SE3(robot.fkine(q, end=robot.links[4]).A)

    Tr = O * sm.SE3.Tx(r[0]) * sm.SE3.Ty(r[1]) * sm.SE3.Tz(r[2])

    n_norm = np.array(n) / np.linalg.norm(n)
    center = np.array(r) + n_norm * 0.05 / 2
    z = np.array([0, 0, 1])
    axis = np.cross(z, n_norm)
    if np.linalg.norm(axis) < 1e-6:
        T_arrow = sm.SE3.Tx(center[0]) * sm.SE3.Ty(center[1]) * sm.SE3.Tz(center[2])
    else:
        angle = np.arccos(np.clip(np.dot(z, n_norm), -1, 1))
        axis = axis / np.linalg.norm(axis)
        T_arrow = sm.SE3(center) * sm.SE3.AngleAxis(angle, axis)

    new_poses = [
        O,                                                      # sphere(O)
        O * sm.SE3.Tz(q[0] / 2),                               # cyl_z(q[0], O)
        T0,                                                     # sphere(T0)
        T0 * sm.SE3.Tx(0.029 / 2) * sm.SE3.Ry(pi / 2),        # cyl_x(0.029, T0)
        T0_tx,                                                  # sphere(T0_tx)
        T0_rx * sm.SE3.Tz(0.0125 / 2),                        # cyl_z(-0.0125, T0_rx)
        T1,                                                     # sphere(T1)
        T1_rx * sm.SE3.Tx(0.029 / 2) * sm.SE3.Ry(pi / 2),     # cyl_x(0.029, T1_rx)
        T2,                                                     # sphere(T2)
        T2_rx * sm.SE3.Tz(0.06823 / 2),                        # cyl_z(0.06823, T2_rx)
        T3,                                                     # sphere(T3)
        T3_roll,                                                # Mesh roll_mechanism
        T3 * sm.SE3.Tx(0.02974 / 2) * sm.SE3.Ry(pi / 2),      # cyl_x(0.02974, T3)
        T3_tx,                                                  # sphere(T3_tx)
        T3_tx * sm.SE3.Tz(q[4] / 2),                           # cyl_z(q[4], T3_tx)
        T4,                                                     # sphere(T4)
        Tr,                                                     # sphere(Tr) red target
        T_arrow,  
    ]

    for shape, pose in zip(shapes, new_poses):
        shape.T = pose.A

knife = Mesh(
    filename=r"C:\Vision\CAD\8IN_GERMAN_CHEF_KNIFE.STL",
    scale=(0.001, 0.001, 0.001),
    pose = (sm.SE3.Ty(0.15)*sm.SE3.Tx(0.09)*sm.SE3.Tz(0.1175)*sm.SE3.Rz(-pi/2)).A,
    color=(1.0, 1.0, 0.0, 1.0)  # RGBA: yellow
)

# ─── Auto zoom ───────────────────────────────────────────────────────────────
def autozoom(env, robot, q, scale=1.5):
    T_ee = robot.fkine(q)
    ee_pos = T_ee.t
    dist = float(np.linalg.norm(ee_pos)) * scale

    # Camera positioned diagonally, looking at origin
    cam_pos = [dist, dist, dist]
    try:
        env.set_camera_pose(cam_pos, [0, 0, 0])
    except AttributeError:
        # Fallback: some swift versions use different method
        print(f"Note: set_camera_pose not available. Manual zoom distance: {dist:.3f}m")

def mm_to_m_vec(v):
    result = [0,0,0]
    for i in range(3):
        result[i] = v[i]/1000.0
    return result

def skew(v):
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def joint_v(robot, r, n, q, 
               lam=0.5,
               mu=1e-3):
    
    n = n / np.linalg.norm(n)
    q = q.copy()
    
    # Forward kinematics
    T = robot.fkine(q)
    p = T.t
    R = T.R
    
    x_axis = R[:, 0]
    
    # ---- Error vector ----
    pos_error = p - r
    orient_error = np.cross(x_axis, n)
    
    e = np.hstack((pos_error, orient_error))
    
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
    qd = - lam * J_pinv @ e

    return qd, e


# ─── Launch ──────────────────────────────────────────────────────────────────
env = swift.Swift()
env.launch()
env.add(robot)
env.add(knife)

q0 = [0,0,pi/2,pi/2,0]
robot.q = q0

autozoom(env, robot, q0, scale=1.0)

data = np.load("knife_data.npz")
_ = data['arr_0']
normals = data['arr_1']
profile = data['arr_2']

r = [0.15,0,0.2]
n = [-0.1,0,0.9]
r = profile[0]
n = normals[0]
r = mm_to_m_vec(r)
n = mm_to_m_vec(n)
n = n/np.linalg.norm(n)
print(r)
print(n)

shapes = build_shapes(q0,r,n)
for s in shapes:
    env.add(s)

thr = 1e-3

# '''
while True:
    robot.q = q0
    qd = [0.0, 0.0, 0.0, 0.0, 0.0]  # joint velocities (m/s or rad/s)

    r = profile[0]
    n = normals[0]
    r = mm_to_m_vec(r)
    n = n/np.linalg.norm(n)

    dt = 0.03
    iter = 0
    while True:
        qd, e = joint_v(robot,r,n,robot.q,lam=0.5)

        if np.linalg.norm(e) < thr:
            break

        # Integrate velocity to get new joint positions
        robot.q = robot.q + qd * dt

        # Clamp to joint limits
        for i, link in enumerate(robot.links):
            if link.qlim is not None:
                robot.q[i] = np.clip(robot.q[i], link.qlim[0], link.qlim[1])

        update_shapes(shapes, robot.q)
        iter += 1
        env.step(dt)

    print("iter = ", iter)

    for i in range(1,len(profile)):
        r = profile[i]
        n = normals[i]
        r = mm_to_m_vec(r)
        n = n/np.linalg.norm(n)

        qd = [0.0, 0.0, 0.0, 0.0, 0.0]  # joint velocities (m/s or rad/s)
        dt = 1

        while True:
            qd, e = joint_v(robot,r,n,robot.q,lam=0.5)

            if np.linalg.norm(e) < thr:
                # print("converged!!!")
                break

            # Integrate velocity to get new joint positions
            robot.q = robot.q + qd * dt

            # Clamp to joint limits
            for i, link in enumerate(robot.links):
                if link.qlim is not None:
                    robot.q[i] = np.clip(robot.q[i], link.qlim[0], link.qlim[1])

            update_shapes(shapes, robot.q)

            # print("error norm = ", np.linalg.norm(e))
            # print("position error = ", np.linalg.norm(e[:2]))
            # print("orientation error = ", np.linalg.norm(e[3:]))
            # print("joint velocities = ", qd)
            # print("end effector position = ", robot.fkine(robot.q))
            # print("joint variables = ", robot.q)

            env.step(dt)

    qd = [0.0, 0.0, 0.0, 0.0, 0.0]  # joint velocities (m/s or rad/s)

    r = robot.fkine(q0).t
    n = [0,0,1]

    dt = 0.03

    while True:
        qd, e = joint_v(robot,r,n,robot.q,lam=0.5)

        if np.linalg.norm(e) < thr:
            break

        # Integrate velocity to get new joint positions
        robot.q = robot.q + qd * dt

        # Clamp to joint limits
        for i, link in enumerate(robot.links):
            if link.qlim is not None:
                robot.q[i] = np.clip(robot.q[i], link.qlim[0], link.qlim[1])

        update_shapes(shapes, robot.q)

        env.step(dt)
'''

p_thr = 0.0005
# o_thr = sin(radians(1))
dt = 1
max_iter = 1000

# while True:
iter = 0
q0 = [robot.links[0].qlim[1],0,pi/2,-pi/2,0]
robot.q = q0
qd = [0.0, 0.0, 0.0, 0.0, 0.0]  # joint velocities (m/s or rad/s)
r =  [0.1095, 0.14597, 0.1315]
n = [ 0.22133524,  0.09916774, 0.9701425 ]
# r = mm_to_m_vec(r)
n = n/np.linalg.norm(n)
# dt = 0.03
while iter < max_iter:
    iter += 1
    qd, e = joint_v(robot,r,n,robot.q,lam=0.5)
    if np.linalg.norm(e) < p_thr:
        break

    print(iter)
    print(np.linalg.norm(e))
    print(robot.q)
    # Integrate velocity to get new joint positions
    robot.q = robot.q + qd * dt
    # Clamp to joint limits
    for i, link in enumerate(robot.links):
        if link.qlim is not None:
            robot.q[i] = np.clip(robot.q[i], link.qlim[0], link.qlim[1])
    update_shapes(shapes, robot.q)
    env.step(dt)
    # print("position error = ", np.linalg.norm(e[:2]))
    # print("orientation error = ", np.linalg.norm(e[3:]))
    # print("joint variables = ", robot.q)
if iter >= max_iter:
    print("did not converge")
else:
    print("converged! in iterations:", iter)

    sleep(1)

'''

env.hold()
