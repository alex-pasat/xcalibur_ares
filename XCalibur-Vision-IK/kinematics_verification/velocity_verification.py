import roboticstoolbox as rtb
import swift
from math import pi
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

ORANGE = (1.0, 0.4, 0.1, 1.0)
BLUE = (0.2, 0.6, 1.0, 1.0)
RED = (1.0, 0.0, 0.0, 1.0)
GREEN = (0.0, 1.0, 0.0, 1.0)

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

def build_shapes(q, r, robot):
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
        # make_normal_arrow(r, length=0.05, color=RED),
    ]

# ─── Update shapes ───────────────────────────────────────────────────────────
def update_shapes(shapes, q, r):
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

    # n_norm = np.array(n) / np.linalg.norm(n)
    # center = np.array(r) + n_norm * 0.05 / 2
    # z = np.array([0, 0, 1])
    # axis = np.cross(z, n_norm)
    # if np.linalg.norm(axis) < 1e-6:
    #     T_arrow = sm.SE3.Tx(center[0]) * sm.SE3.Ty(center[1]) * sm.SE3.Tz(center[2])
    # else:
    #     angle = np.arccos(np.clip(np.dot(z, n_norm), -1, 1))
    #     axis = axis / np.linalg.norm(axis)
    #     T_arrow = sm.SE3(center) * sm.SE3.AngleAxis(angle, axis)

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
        # T_arrow,  
    ]

    for shape, pose in zip(shapes, new_poses):
        shape.T = pose.A

knife = Mesh(
    filename=r"C:\Vision\CAD\8IN_GERMAN_CHEF_KNIFE.STL",
    scale=(0.001, 0.001, 0.001),
    pose = (sm.SE3.Ty(0.15)*sm.SE3.Tx(0.09)*sm.SE3.Tz(0.1185)*sm.SE3.Rz(-pi/2)).A,
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

def convert_q(q):
    result = q.copy()
    result[0] = result[0]/1000
    result[4] = result[4]/1000
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

def q_err(robot, q_dest):
    e = q_dest - robot.q
    
    return e

def go_dest(robot, q_dest, r, n, steps=50, dt=0.01):
    q_start = robot.q.copy()
    q_dest = np.array(q_dest)
    
    for i in range(steps + 1):
        alpha = i / steps  # 0 to 1
        robot.q = q_start + alpha * (q_dest - q_start)
        update_shapes(shapes, robot.q, r)
        env.step(dt)

if __name__ == "__main__":

    # ─── Launch ──────────────────────────────────────────────────────────────────
    env = swift.Swift()
    env.launch(realtime=True)
    env.add(robot)
    env.add(knife)

    q0 = [0,0,pi/2,pi/2,0]
    robot.q = q0

    data = np.load("knife_data.npz")
    tip_q1 = data['tip_q1']           # joint variables to reach knife tip on first side
    tip_q1 = data['tip_q1']           # joint variables to reach knife tip on second side
    yaw_indices = data['yaw_indices'] # yaw values of sample points (note they are descending)
    ratios1 = data['ratios1']         # ratio between actuator velocities and yaw velocity for first side
    ratios2 = data['ratios2']         # ratio between actuator velocities and yaw velocity for second side

    q0 = tip_q1
    r = robot.fkine(q0).t
    r = mm_to_m_vec(r)

    shapes = build_shapes(q0,r,robot)
    for s in shapes:
        env.add(s)

    # ratio slice is an array with 4 elements for the specific yaw interval
    def apply_ratio(yaw_v, ratio_slice):
        velocity = yaw_v * ratio_slice # multiply other actuator velocities by appropriate ratios
        velocity = [*velocity[:2], *[yaw_v], *velocity[2:]] # middle index is just the yaw velocity
        return np.array(velocity)  

    yaw_v = -pi/4 # set yaw velocity rad/s
    dt = 0.005

    while True:
        # start at hilt
        robot.q = q0
        yaw_idx = 0

        # while the yaw hasn't reached the hilt
        while robot.q[2] > yaw_indices[-1]:
            # check if we got to next yaw index
            if robot.q[2] <= yaw_indices[yaw_idx+1]:
                yaw_idx += 1
            
            # with correct idx, get velocity and update
            velocity = apply_ratio(yaw_v, ratios1[yaw_idx])
            qd = velocity

            robot.q = robot.q + dt*qd

            pose = robot.fkine(robot.q)
            update_shapes(shapes, robot.q, pose.t)
            env.step(dt)

        sleep(0.5)

    env.hold()
