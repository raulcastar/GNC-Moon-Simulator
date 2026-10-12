"""ROS message construction for the planner node."""

import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


def path_msg(points, stamp, frame_id: str = 'map') -> Path:
    """A nav_msgs/Path through `points` ([[x, y], ...]); empty points give an empty path."""
    msg = Path()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    for pt in points:
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = float(pt[0])
        ps.pose.position.y = float(pt[1])
        msg.poses.append(ps)
    return msg


def matrix_msg(matrix: np.ndarray) -> Float32MultiArray:
    """A 2D array as Float32MultiArray, with its height and width in the layout."""
    msg = Float32MultiArray()
    dim_height = MultiArrayDimension()
    dim_height.label = "height"
    dim_height.size = matrix.shape[0]
    dim_height.stride = matrix.shape[0] * matrix.shape[1]
    dim_width = MultiArrayDimension()
    dim_width.label = "width"
    dim_width.size = matrix.shape[1]
    dim_width.stride = matrix.shape[1]
    msg.layout.dim = [dim_height, dim_width]
    msg.data = matrix.flatten().tolist()
    return msg


def finite_cost_map(cost_map: np.ndarray) -> np.ndarray:
    """The cost map with infinite (obstacle) cells as 9999, for IEEE-portable messages."""
    safe = np.copy(cost_map)
    safe[np.isinf(safe)] = 9999.0
    return safe
