import math
import numpy as np
import matplotlib.pyplot as plt
from knifePlane import tangent, blade_profile_smooth, blade_profile

x_orig = blade_profile[:,0,0]
y_orig = blade_profile[:,0,1]

x_data = blade_profile_smooth[:,0,0]
y_data = blade_profile_smooth[:,0,1]

fig, ax = plt.subplots()

# Original profile
ax.plot(x_orig, y_orig, linestyle='--', alpha=0.5, label="Original")

# Smoothed profile
ax.plot(x_data, y_data, linewidth=2, label="Smoothed")

tangent_line, = ax.plot([0, 0], [0, 0], linewidth=2, label="Tangent")
point_marker, = ax.plot([0], [0], marker='o')

# 🔹 Angle text (top-left corner of axes)
angle_text = ax.text(
    0.02, 0.95, "",
    transform=ax.transAxes,
    verticalalignment='top'
)

V = tangent(blade_profile_smooth)

def on_mouse_move(event):
    if event.inaxes != ax or event.xdata is None:
        return

    i = np.argmin(np.abs(x_data - event.xdata))

    if i <= 0 or i >= len(blade_profile_smooth) - 1:
        return
    
    v = V[i]

    x0 = x_data[i]
    y0 = y_data[i]

    # ---- Compute angle ----
    angle_rad = math.atan2(v[1], v[0])
    angle_deg = math.degrees(angle_rad)

    x_min = x_data.min()
    x_max = x_data.max()

    if abs(v[0]) > 1e-8:
        slope = v[1] / v[0]
        tangent_x = np.array([x_min, x_max])
        tangent_y = y0 + slope * (tangent_x - x0)
    else:
        tangent_x = np.array([x0, x0])
        tangent_y = np.array([y_data.min(), y_data.max()])

    tangent_line.set_data(tangent_x, tangent_y)
    point_marker.set_data([x0], [y0])

    # 🔹 Update angle display
    angle_text.set_text(f"Tangent angle: {angle_deg:.2f}°")

    fig.canvas.draw_idle()

fig.canvas.mpl_connect("motion_notify_event", on_mouse_move)

plt.xlabel("x")
plt.ylabel("y")
plt.title("Interactive Tangent Viewer")
plt.legend()
plt.show()

