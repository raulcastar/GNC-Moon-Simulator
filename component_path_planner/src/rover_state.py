"""The rover's state from odometry: attitude, Unreal session jumps, the driven trajectory. No ROS."""

import math

# Driven trajectory points are kept this far apart (metres).
TRAJECTORY_SPACING_M = 0.1


def yaw_from_quaternion(q) -> float:
    """Yaw (rad) of a geometry_msgs-like quaternion (x, y, z, w attributes)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def tilt_deg(q) -> float:
    """Combined roll/pitch tilt (deg): the angle between the body z axis and vertical."""
    cz = max(-1.0, min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y)))
    return math.degrees(math.acos(cz))


def is_session_jump(previous_xy, x: float, y: float, limit_m: float) -> bool:
    """
    Whether the rover moved more than limit_m in one odometry step: a Play/Stop
    in Unreal teleports it back to its start pose.
    """
    return (previous_xy is not None
            and math.hypot(x - previous_xy[0], y - previous_xy[1]) > limit_m)


def record_point(history: list, x: float, y: float, spacing: float = TRAJECTORY_SPACING_M) -> bool:
    """Appends (x, y) to the driven trajectory if it is `spacing` from the last point."""
    if not history or math.hypot(x - history[-1][0], y - history[-1][1]) > spacing:
        history.append([x, y])
        return True
    return False
