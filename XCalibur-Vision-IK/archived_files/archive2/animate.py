import roboticstoolbox as rtb
import numpy as np
from math import pi, degrees
from matplotlib import pyplot as plt
import time

data = np.load("knife_data.npz")
q = data["arr_0"]

def robot():
    # Define the links
    # First, the underpass
    L1 = rtb.PrismaticMDH(theta=0, a=0, alpha=0, qlim=[0, 198])
    L2 = rtb.RevoluteMDH(a=29, alpha=-pi/2, d=-12.5, qlim=[0, 2*pi])
    L3 = rtb.RevoluteMDH(a=27.5, alpha=pi/2, d=-0, qlim=[0, 2*pi])
    L4 = rtb.RevoluteMDH(a=0, alpha=pi/2, d=68.23, qlim=[0, 2*pi])
    L5 = rtb.PrismaticMDH(theta=0, a=29.74, alpha=0, qlim=[0, 95])

    # Create robot
    robot = rtb.DHRobot([L1, L2, L3, L4, L5], name="Robot")

    return robot

def animate_robot(robot, result, dt=0.01):

    q_traj = np.array(result)

    robot.plot(q_traj, dt=dt, block=False)

    # compute tool path
    pts = []
    for q in q_traj:
        T = robot.fkine(q)
        pts.append(T.t)

    pts = np.array(pts)

    import matplotlib.pyplot as plt
    ax = plt.gca()
    ax.plot(pts[:,0], pts[:,1], pts[:,2], 'r')

animate_robot(robot(),q)