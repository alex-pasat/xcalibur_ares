import math
import numpy as np
import matplotlib.pyplot as plt
from knifePlane import tangent, blade_profile_smooth

# blade_profile_smooth already computed
x_data = blade_profile_smooth[:,0,0]
y_data = blade_profile_smooth[:,0,1]

# Prepare array to store angles
angles_deg = np.zeros(len(x_data))

V = tangent(blade_profile_smooth)

diff = len(x_data) - len(V)

# Compute tangent angle at each valid point (avoid edges where samples can't be used)
for i in range(len(x_data)):
    if i in range(diff, diff+len(V)):
        v = V[i]
        angle_rad = math.atan2(v[1], v[0])
        angles_deg[i] = math.degrees(angle_rad)
    else:
        angles_deg[i] = 0

# Plot
plt.figure(figsize=(10,5))
plt.plot(x_data, angles_deg, label="Tangent Angle (°)")
plt.xlabel("x position")
plt.ylabel("Tangent angle (degrees)")
plt.title("Tangent Angle vs X Position for Smoothed Blade Profile")
plt.grid(True)
plt.legend()
plt.show()
