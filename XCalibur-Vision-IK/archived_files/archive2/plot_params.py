import roboticstoolbox as rtb
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

data = np.load("knife_data.npz")
q0 = data['arr_0']
normals = data['arr_1']
profile = data['arr_2']

normals_x = normals[:,0]
plt.plot(profile[:,1],normals_x)
plt.show()

# print(q0)

# q = []
# for i in range(5):
#     q.append([q0[j][i] for j in range(len(q0))])

# x = q[2][:]
# y = q[0][:]

# fig, ax = plt.subplots()

# ax.scatter(x, y, color='gray')
# dot, = ax.plot([], [], 'ro')

# ax.set_xlabel("Yaw")
# ax.set_ylabel("Underpass")

# def update(frame):
#     dot.set_data([x[frame]], [y[frame]])
#     return dot,

# ani = FuncAnimation(fig, update, frames=len(x), interval=30, blit=True)

# plt.show()

# def robot():
#     # Define the links
#     # First, the underpass
#     L1 = rtb.PrismaticMDH(theta=0, a=0, alpha=0, qlim=[0, 198])
#     L2 = rtb.RevoluteMDH(a=29, alpha=-pi/2, d=-12.5, qlim=[0, 2*pi])
#     L3 = rtb.RevoluteMDH(a=27.5, alpha=pi/2, d=-0, qlim=[0, 2*pi])
#     L4 = rtb.RevoluteMDH(a=0, alpha=pi/2, d=68.23, qlim=[0, 2*pi])
#     L5 = rtb.PrismaticMDH(theta=0, a=29.74, alpha=0, qlim=[0, 95])

#     # Create robot
#     robot = rtb.DHRobot([L1, L2, L3, L4, L5], name="Robot")

#     return robot

# robot = robot()

# pos = []

# for i in range(len(q0)):
#     T = robot.fkine(q0[i])
#     p = T.t
#     pos.append(p)
#     print(p)

# # convert to array for easier slicing
# import numpy as np
# pos = np.array(pos)

# plt.plot(pos[:,0], pos[:,1])
# plt.xlabel("x")
# plt.ylabel("y")
# plt.show()
