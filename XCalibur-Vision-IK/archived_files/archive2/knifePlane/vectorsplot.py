import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from knifePlane import blade_profile_smooth, bevels  # tangents optional

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Extract blade points
X = blade_profile_smooth[:,0,0]
Y = blade_profile_smooth[:,0,1]
Z = np.zeros_like(X)  # profile lies in XY plane

# Plot blade profile
ax.plot(X, Y, Z, color='black', lw=2, label='Blade Profile')

# Normalize bevel vectors
B = bevels / np.linalg.norm(bevels, axis=1)[:, None]
print(B)

# Scale for visualization
scale = 0.01  # adjust for visibility
U = B[:,0] * scale
V = B[:,1] * scale
W = B[:,2] * scale

# Plot bevel vectors as 3D arrows using quiver
ax.quiver(X, Y, Z, B[:,0], B[:,1], B[:,2], length=scale, normalize=True, color='green')

# Optionally, plot tangents too (projected in XY plane)
# from knifePlane import tangents
# T = tangents / np.linalg.norm(tangents, axis=1)[:, None]
# ax.quiver(X, Y, Z, T[:,0], T[:,1], T[:,2], length=1.0, normalize=True, color='red', linewidth=1)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Blade Profile with Unit Bevel Vectors")
ax.legend()

# Adjust view for better 3D visualization
ax.view_init(elev=30, azim=-60)
plt.show()