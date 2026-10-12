"""ROS message construction for the stereo node's camera products."""

from typing import Tuple

import numpy as np
from builtin_interfaces.msg import Time as RosTime
from sensor_msgs.msg import CameraInfo, RegionOfInterest
from stereo_msgs.msg import DisparityImage


def ros_time(ns: int) -> RosTime:
    """Simulation time in ns (UnrealSimClock) -> builtin_interfaces/Time."""
    return RosTime(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))


def camera_info(stamp, frame_id: str, intrinsics: Tuple[float, float, float, int, int],
                baseline_m: float = 0.0) -> CameraInfo:
    """
    Pinhole CameraInfo for an already-rectified camera, from (fx, cx, cy,
    width, height). A right eye passes its baseline: P[3] = -fx * B, which is
    how the standard ROS stereo pipeline recovers depth from disparity.
    """
    fx, cx, cy, width, height = intrinsics
    info = CameraInfo()
    info.header.stamp = stamp
    info.header.frame_id = frame_id
    info.width = width
    info.height = height
    info.distortion_model = 'plumb_bob'
    info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    info.k = [fx, 0.0, cx,
              0.0, fx, cy,
              0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0]
    info.p = [fx, 0.0, cx, -fx * baseline_m if baseline_m else 0.0,
              0.0, fx, cy, 0.0,
              0.0, 0.0, 1.0, 0.0]
    return info


def disparity_msg(bridge, disp: np.ndarray, stamp, frame_id: str, fx: float,
                  baseline_m: float, num_disparities: int, width: int, height: int,
                  border_columns: int = 0) -> DisparityImage:
    """
    DisparityImage. Unmatched pixels go out as 0.0: consumers treat anything
    <= min_disparity as invalid, and NaN is not portable here. border_columns
    > 0 marks the leftmost columns, which have no search window in the right
    image, as outside the valid window; otherwise validity is per pixel.
    """
    msg = DisparityImage()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.image = bridge.cv2_to_imgmsg(np.nan_to_num(disp, nan=0.0), encoding='32FC1')
    msg.image.header.stamp = stamp
    msg.image.header.frame_id = frame_id
    msg.f = float(fx)
    msg.t = float(baseline_m)
    msg.min_disparity = 0.0
    msg.max_disparity = float(num_disparities)
    msg.delta_d = 1.0 / 16.0    # SGBM's sub-pixel quantum
    msg.valid_window = RegionOfInterest(
        x_offset=border_columns, y_offset=0, width=max(0, width - border_columns),
        height=height, do_rectify=False)
    return msg
