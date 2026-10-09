"""
Lidar versus ground truth: one lidar/points scan projected into the central
navcam and compared with Unreal's own depth (navcam/central/depth_ground_truth).
Reports the scan rate, return coverage and the depth error near and far.

Runs inside the component_stereo_depth container during Play, with the lidar
following the central navcam (UnrealLidarCam FollowComponentName):
    tools/scenarios/run.sh lidar_gt [seconds]
"""
import sys, time
import numpy as np
import rclpy
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge

got = {}
br = CvBridge()
rclpy.init()
n = rclpy.create_node("lidar_vs_gt")
n.create_subscription(PointCloud2, "/lidar/points", lambda m: got.__setitem__("cloud", m), 10)
n.create_subscription(Image, "/navcam/central/depth_ground_truth", lambda m: got.__setitem__("gt", br.imgmsg_to_cv2(m)), 10)
counts = {"cloud": 0}
n.create_subscription(PointCloud2, "/lidar/points", lambda m: counts.__setitem__("cloud", counts["cloud"] + 1), 10)
t0 = time.time()
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
while time.time() - t0 < DURATION:
    rclpy.spin_once(n, timeout_sec=0.05)
print("clouds received: %d in %.0f s (%.1f Hz)" % (counts["cloud"], time.time() - t0, counts["cloud"] / (time.time() - t0)))
if "cloud" not in got or "gt" not in got:
    print("missing:", [k for k in ("cloud", "gt") if k not in got]); sys.exit(1)
m, gt = got["cloud"], got["gt"]
dt = np.dtype({"names": ["x", "y", "z", "i", "ring", "col"], "formats": ["<f4"] * 4 + ["<u2"] * 2,
               "offsets": [0, 4, 8, 12, 16, 18], "itemsize": 20})
a = np.frombuffer(bytes(m.data), dt)
print(f"cloud: {m.height} x {m.width}, frame {m.header.frame_id}, returns {np.isfinite(a['x']).sum()} "
      f"({100 * np.isfinite(a['x']).mean():.1f}%), intensity {np.nanmin(a['i']):.1f}-{np.nanmax(a['i']):.1f}")
ok = np.isfinite(a["x"]) & (a["x"] > 0)
x, y, z = a["x"][ok], a["y"][ok], a["z"][ok]
H, W = gt.shape
f = W / 2.0                                   # 90 deg horizontal FOV
u = (W / 2.0 - f * y / x).astype(int)          # ROS y left -> image left
v = (H / 2.0 - f * z / x).astype(int)
inside = (u >= 0) & (u < W) & (v >= 0) & (v < H)
g = gt[v[inside], u[inside]]
fin = np.isfinite(g)
err = x[inside][fin] - g[fin]                  # Lidar planar depth (x) vs ground-truth planar depth
print(f"points in the image: {inside.sum()} | with ground truth: {fin.sum()}")
print(f"depth difference: median {np.median(err):+.4f} m | median |d| {np.median(abs(err)):.4f} m | "
      f"90% |d| {np.percentile(abs(err), 90):.4f} m | relative median {100 * np.median(abs(err) / g[fin]):.2f}%")
far = g[fin] > 6
print(f"  < 6 m: median |d| {np.median(abs(err[~far])):.4f} m over {(~far).sum()} pts; "
      f">= 6 m: median |d| {np.median(abs(err[far])) if far.any() else float('nan'):.3f} m over {far.sum()} pts")
sky_gt = np.isinf(gt).mean()
print(f"sky: ground truth {100 * sky_gt:.1f}% of pixels | lidar no-return {100 * (1 - np.isfinite(a['x']).mean()):.1f}% of beams")
