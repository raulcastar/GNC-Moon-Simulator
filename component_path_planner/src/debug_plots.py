"""
Optional matplotlib windows of the planner node (enable_path_visualization):
a preview of the maps at configure time and a live plot of the drive.
"""

import math

import numpy as np
import matplotlib.pyplot as plt


def preview_maps(elevation: np.ndarray, cost: np.ndarray, goal=None) -> None:
    """Elevation and cost maps side by side, for checking their alignment."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    im1 = ax1.imshow(elevation, origin='lower', cmap='viridis')
    ax1.set_title("Full Elevation Map (Cartesian Aligned)")
    ax1.set_xlabel("X (Cols)")
    ax1.set_ylabel("Y (Rows)")
    fig.colorbar(im1, ax=ax1, label="Elevation (Meters)")

    im2 = ax2.imshow(cost, origin='lower', cmap='hot')
    ax2.set_title("Full Cost Map (Obstacles & Slopes)")
    ax2.set_xlabel("X (Cols)")
    ax2.set_ylabel("Y (Rows)")
    fig.colorbar(im2, ax=ax2, label="Cost Penalty")

    if goal is not None:
        ax1.plot(goal[0], goal[1], 'xb', markersize=12, markeredgewidth=3, label="Goal")
        ax2.plot(goal[0], goal[1], 'xb', markersize=12, markeredgewidth=3, label="Goal")

    ax1.legend()
    ax2.legend()

    plt.tight_layout()
    plt.show(block=False)
    plt.pause(3.0)


def update_live_plot(config, robot_pose, elevation, global_path, predicted_trajectory,
                     trajectory_history, obstacles, goal) -> None:
    """Redraws the live plot: maps, routes, the rover and its footprint."""
    plt.cla()

    # ESC in the plot window exits the node.
    plt.gcf().canvas.mpl_connect(
        'key_release_event',
        lambda event: [exit(0) if event.key == 'escape' else None])

    if elevation is not None:
        plt.contourf(elevation, levels=20, cmap='viridis', alpha=0.5, origin='lower')

    if global_path is not None and len(global_path) > 0:
        plt.plot(global_path[:, 0], global_path[:, 1], "-r", linewidth=2, label="Global Path")

    if predicted_trajectory is not None and len(predicted_trajectory) > 0:
        plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g", linewidth=2,
                 label="Local DWA Path")

    if len(trajectory_history) > 0:
        hist = np.array(trajectory_history)
        plt.plot(hist[:, 0], hist[:, 1], "-b", label="Actual Trajectory")

    plt.plot(robot_pose[0], robot_pose[1], "xr")
    if goal is not None:
        plt.plot(goal[0], goal[1], "xb", markersize=10, label="Target Goal")

    if len(obstacles) > 0:
        plt.plot(obstacles[:, 0], obstacles[:, 1], "sk")

    plot_footprint(robot_pose[0], robot_pose[1], robot_pose[2], config.robot_length, config.robot_width)
    plt.arrow(robot_pose[0], robot_pose[1], 0.5 * math.cos(robot_pose[2]), 0.5 * math.sin(robot_pose[2]),
              head_length=0.1, head_width=0.1)

    if elevation is not None:
        plt.xlim(0, elevation.shape[1])
        plt.ylim(0, elevation.shape[0])

    plt.grid(True)
    plt.legend(loc='upper right')
    plt.pause(0.001)


def plot_footprint(x, y, yaw, length, width) -> None:
    """The rover's rectangular footprint at (x, y, yaw)."""
    corners = np.array([[-length / 2, length / 2, length / 2, -length / 2, -length / 2],
                        [width / 2, width / 2, -width / 2, -width / 2, width / 2]])
    rotation = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    corners = (corners.T.dot(rotation)).T
    plt.plot(corners[0, :] + x, corners[1, :] + y, "-k")


def close_all() -> None:
    plt.close('all')


def interactive() -> None:
    plt.ion()
