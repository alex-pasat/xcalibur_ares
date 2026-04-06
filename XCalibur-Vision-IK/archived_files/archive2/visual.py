import roboticstoolbox as rtb
import swift
from math import pi
from spatialgeometry import Cylinder, Sphere
import spatialmath as sm
import numpy as np

RADIUS = 4  # mm

def cyl_x(length, offset=sm.SE3()):
    pose = offset * sm.SE3.Tx(length / 2) * sm.SE3.Ry(pi / 2)
    return Cylinder(radius=RADIUS, length=abs(length), pose=pose.A)

def cyl_z(length, offset=sm.SE3()):
    pose = offset * sm.SE3.Tz(length / 2)
    return Cylinder(radius=RADIUS, length=abs(length), pose=pose.A)

def joint_sphere():
    return Sphere(radius=RADIUS * 1.5)

class XCalibur(rtb.DHRobot):
    def __init__(self):
        links = [
            rtb.PrismaticMDH(
                theta=0, a=0, alpha=0, qlim=[0, 198],
                geometry=[cyl_z(20)]
            ),
            rtb.RevoluteMDH(
                a=29, alpha=-pi/2, d=-12.5, qlim=[0, 2*pi],
                geometry=[
                    joint_sphere(),
                    cyl_x(29),
                    cyl_z(-12.5, offset=sm.SE3.Tx(29)),
                ]
            ),
            rtb.RevoluteMDH(
                a=27.5, alpha=pi/2, d=0, qlim=[0, 2*pi],
                geometry=[
                    joint_sphere(),
                    cyl_x(27.5),
                ]
            ),
            rtb.RevoluteMDH(
                a=0, alpha=pi/2, d=68.23, qlim=[0, 2*pi],
                geometry=[
                    joint_sphere(),
                    cyl_z(68.23),
                ]
            ),
            rtb.PrismaticMDH(
                theta=0, a=29.74, alpha=0, qlim=[0, 95],
                geometry=[
                    joint_sphere(),
                    cyl_x(29.74),
                ]
            ),
        ]

        super().__init__(links, name="XCalibur")
        self.q = [0, 0, 0, 0, 0]

# ─── Usage ───────────────────────────────────────────────────────────────────
robot = XCalibur()
robot.q = [0, 0, 0, 0, 0]

# Get end-effector position
T_ee = robot.fkine(robot.q)
ee_pos = T_ee.t  # [x, y, z] in mm

# Distance from base to end-effector
dist = np.linalg.norm(ee_pos)
camera_dist = dist * 1.5

env = swift.Swift()
env.launch(zoom=camera_dist)
env.add(robot)

env.hold()