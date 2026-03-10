import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

file_path = "/home/gonazza/DroneMission_ws/positions_log.csv"

data = pd.read_csv(file_path)

t  = data["time"].to_numpy()

x1 = data["x1"].to_numpy()
y1 = data["y1"].to_numpy()
z1 = data["z1"].to_numpy()

x2 = data["x2"].to_numpy()
y2 = data["y2"].to_numpy()
z2 = data["z2"].to_numpy()

# -----------------------------
# PUNTI EDITABILI
# -----------------------------

# Waypoints (X marker)
waypoints = [
    (-3.0, 3.0, 0.3),
    (1.5, 3.0, 0.3),
    # (0.0, 0.0, 0.3),
    # (-3.0, 1.0, 0.3),
    # (1.0, 1.5, 0.3),
    # (0.0, 4.0, 0.3)
]

# Ostacoli (O marker)
obstacles = [
	        (-2.0, 2.0, 0.1),
            (1.5, 2.0, 0.1),
            # (1.0, -1.0, 0.1),
            # (-3.0, 0.0, 0.1),
            # (1.0, 4.0, 0.1),
            # (-1.0, 2.0, 0.1),
            # (1.5, 3.0, 0.1),
            # (0.0, -2.0, 0.1),
            # (1.5, 3.0, 0.1),
            # (-1.5, -1.5, 0.1),
]

# separazione coordinate
wx, wy, wz = zip(*waypoints) if waypoints else ([], [], [])
ox, oy, oz = zip(*obstacles) if obstacles else ([], [], [])

# -----------------------------

fig = plt.figure(figsize=(12,5))

# XY plot
ax1 = fig.add_subplot(121)
ax1.plot(x1, y1, label="Drone 1")
ax1.plot(x2, y2, label="Drone 2")

# waypoint e ostacoli
ax1.scatter(wx, wy, marker='x', s=100, label="Waypoints")
ax1.scatter(ox, oy, marker='o', s=100, label="Obstacles")

ax1.set_xlabel("X")
ax1.set_ylabel("Y")
ax1.set_title("XY trajectory")
ax1.grid()
ax1.legend()

# 3D plot
ax2 = fig.add_subplot(122, projection='3d')
ax2.plot(x1, y1, z1, label="Drone 1")
ax2.plot(x2, y2, z2, label="Drone 2")

# waypoint e ostacoli
ax2.scatter(wx, wy, wz, marker='x', s=100, label="Waypoints")
ax2.scatter(ox, oy, oz, marker='o', s=100, label="Obstacles")

ax2.set_xlabel("X")
ax2.set_ylabel("Y")
ax2.set_zlabel("Z")

ax2.set_title("3D trajectory")
ax2.legend()

plt.tight_layout()
plt.show()