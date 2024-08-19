import math

from matplotlib import ticker
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize, LinearSegmentedColormap
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

from KlippySim import ToolHead, avgspeed


def get_circle_coord(theta, x_center, y_center, radius):
    x = radius * math.cos(theta) + x_center
    y = radius * math.sin(theta) + y_center
    return x, y


def get_all_circle_coords(x_center, y_center, radius, n_points):
    thetas = [i / n_points * math.tau for i in range(n_points)]
    return [get_circle_coord(theta, x_center, y_center, radius) for theta in thetas]


circle_coords = get_all_circle_coords(x_center=640,
                                      y_center=480,
                                      radius=2,
                                      n_points=25)


def get_max_speed(tpath):
    speeds = [x[1][0] for x in tpath] + [x[1][0] for x in tpath]
    return math.ceil(max(speeds) + 1)


toolhead = ToolHead(max_velocity=500, max_accel=15000, mcr=0.0, scv=20.0)
toolhead.set_position((640 + 2, 480))

for coord in circle_coords:
    toolhead.move(coord, 500)

toolhead.flush_lookahead()

fig, ax = plt.subplots(figsize=(16, 16))
plt.xlabel('X, mm')
plt.ylabel('Y, mm')

colors = ["blue", "red"]


def plot_gradient_line(point1, point2, color1, color2):
    x1, y1 = point1
    x2, y2 = point2

    num_points = 100
    x = np.linspace(x1, x2, num_points)
    y = np.linspace(y1, y2, num_points)

    gradient_colors = np.linspace(color1, color2, num_points)

    colormap = plt.get_cmap('plasma')
    colors = colormap(gradient_colors)[:, :3]

    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    lc = LineCollection(segments, colors=colors, linewidth=3)
    ax.add_collection(lc)


def render_toolpath(tpath):
    max_speed = get_max_speed(tpath)

    sm = ScalarMappable(cmap='plasma', norm=Normalize(vmin=0, vmax=max_speed))
    sm.set_array([])  # Only needed for the colorbar
    cbar = plt.colorbar(sm, ax=ax, orientation='vertical')
    cbar.locator = ticker.MaxNLocator(nbins=20)
    cbar.set_label('Speed, mm/s')

    for move in toolhead.lookahead.output:
        color_start = move[1][0] / max_speed
        color_end = move[1][1] / max_speed
        plot_gradient_line(move[0][0], move[0][1], color_start, color_end)


render_toolpath(toolhead.lookahead.output)
avg_spd = round(sum(avgspeed[1:-1]) / len(avgspeed[1:-1]), 2)
plt.figtext(0.5, 0.01, f"MaxSpeed={toolhead.max_velocity} Accel={toolhead.max_accel} SCV={toolhead.square_corner_velocity} MCR={toolhead.min_cruise_ratio} AvgSpeed={avg_spd}", ha="center", fontsize=18, bbox={"facecolor":"orange", "alpha":0.5, "pad":5})
ax.autoscale()
plt.gca().set_aspect('equal')
plt.show()
