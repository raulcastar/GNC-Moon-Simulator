"""
Overlay check: compares the lidar overlay with the stereo local DEM, as the dashboard draws them.

Every PERIOD s: latest DEM (local_mapping/elevation_map_raw), lidar cloud and odom -> the dashboard's
DEM-frame lidar points (pitch -20, levelled with odom roll/pitch, + lidar_height_m), then per DEM cell:
height difference lidar vs DEM, and obstacle agreement (relief above the DEM ground plane >= 0.15 m).
Writes /tmp/overlay/NN.png (DEM relief + lidar points + navcam) and /tmp/overlay/stats.json.

Runs inside the mission_control_dashboard container during Play (drive the rover meanwhile):
    tools/scenarios/run.sh overlay [seconds] [period]
"""
import json, math, os, sys, time
import numpy as np
import cv2
import rclpy
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Float32MultiArray

sys.path.insert(0, '/opt/ros/jazzy/src/mission_control_dashboard')
from src.geometry import ground_plane, level_points as level  # noqa: E402  (what the dashboard draws)

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 150.0
PERIOD = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
LIDAR_PITCH, LIDAR_H, OBST, SELF_R = -20.0, 0.55, 0.15, 0.6
os.makedirs("/tmp/overlay", exist_ok=True)
last = {}
DT = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3, "offsets": [0, 4, 8], "itemsize": 20})


def on_odom(m):
    q = m.pose.pose.orientation
    last["att"] = (math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
                   math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x)))))
    last["pos"] = (m.pose.pose.position.x, m.pose.pose.position.y)


def on_img(m):
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, -1)
    last["img"] = cv2.cvtColor(a[:, :, :3], cv2.COLOR_RGB2BGR) if m.encoding.lower().startswith("rgb") else a[:, :, :3].copy()


rclpy.init()
n = rclpy.create_node("overlay_check")
n.create_subscription(Odometry, "/odom", on_odom, 10)
n.create_subscription(PointCloud2, "/lidar/points", lambda m: last.__setitem__("cloud", (time.time(), m)), 2)
n.create_subscription(Float32MultiArray, "/local_mapping/elevation_map_raw", lambda m: last.__setitem__("dem", (time.time(), m)), 2)
n.create_subscription(OccupancyGrid, "/local_mapping/elevation_grid", lambda m: last.__setitem__("res", m.info.resolution), 2)
n.create_subscription(Image, "/navcam/central/image_color", on_img, 2)


def snapshot(k):
    (td, dm), (tc, cm) = last["dem"], last["cloud"]
    W, H = dm.layout.dim[0].size, dm.layout.dim[1].size
    dem = np.asarray(dm.data, np.float32).reshape(W, H).T     # rows forward, cols lateral (right +)
    res = last.get("res", 0.05)
    a = np.frombuffer(bytes(cm.data), DT); ok = np.isfinite(a["x"])
    xyz = np.c_[a["x"][ok], a["y"][ok], a["z"][ok]].astype(float)
    p = -math.radians(LIDAR_PITCH)
    rover = xyz @ np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]]).T
    lev = level(rover, *last["att"])
    x, y, z = -lev[:, 1], lev[:, 0], lev[:, 2] + LIDAR_H
    col = np.floor(x / res + (W - 1) / 2.0 + 0.5).astype(int); row = np.floor(y / res).astype(int)
    keep = (row >= 0) & (row < H) & (col >= 0) & (col < W) & (np.hypot(lev[:, 0], lev[:, 1]) > SELF_R)
    row, col, z = row[keep], col[keep], z[keep]
    # Lidar height per cell (max, like an obstacle map would use).
    lid = np.full(dem.shape, np.nan, np.float32)
    np.fmax.at(lid, (row, col), z.astype(np.float32))
    plane = ground_plane(dem, res)
    rr, cc = np.indices(dem.shape)
    gp = plane[0] * (cc - (W - 1) / 2.0) * res + plane[1] * (rr + 0.5) * res + plane[2]
    dem_rel, lid_rel = dem - gp, lid - gp
    both = np.isfinite(dem) & np.isfinite(lid)
    d = (lid - dem)[both]
    do, lo = both & (dem_rel >= OBST), both & (lid_rel >= OBST)
    s = dict(k=k, t_age_dem=round(time.time() - td, 2), t_age_cloud=round(time.time() - tc, 2), pos=[round(v, 2) for v in last["pos"]],
             dem_observed=round(float(np.isfinite(dem).mean()), 3), lidar_cells=int(np.isfinite(lid).sum()), both=int(both.sum()),
             dz_median=round(float(np.median(d)), 3) if d.size else None, dz_abs_median=round(float(np.median(abs(d))), 3) if d.size else None,
             dz_abs_p90=round(float(np.percentile(abs(d), 90)), 3) if d.size else None,
             ground_dz_median=round(float(np.median((lid - dem)[both & ~lo & ~do])), 3) if (both & ~lo & ~do).any() else None,
             obst_lidar=int(lo.sum()), obst_dem=int(do.sum()), obst_both=int((lo & do).sum()),
             obst_lidar_only_dem_unseen=int((np.isfinite(lid) & (lid_rel >= OBST) & ~np.isfinite(dem)).sum()),
             lidar_rel_max=round(float(np.nanmax(lid_rel)), 2) if np.isfinite(lid_rel).any() else None,
             dem_rel_max=round(float(np.nanmax(dem_rel)), 2) if np.isfinite(dem_rel).any() else None)
    # Picture: DEM relief (blue 0 -> red >= 0.5 m, grey unobserved), forward = up; lidar cells as dots.
    S = 4
    rel = np.clip(np.nan_to_num(dem_rel, nan=0) / 0.5, 0, 1)
    img = cv2.applyColorMap((rel * 255).astype(np.uint8), cv2.COLORMAP_JET)
    img[~np.isfinite(dem)] = (60, 60, 60)
    img[np.isfinite(dem) & (dem_rel >= OBST)] = (0, 0, 255)
    img = cv2.resize(img[::-1], (W * S, H * S), interpolation=cv2.INTER_NEAREST)
    for r_, c_ in zip(*np.nonzero(np.isfinite(lid))):
        colr = (255, 0, 255) if lid_rel[r_, c_] >= OBST else (255, 255, 255)
        cv2.circle(img, (int(c_ * S + S / 2), int((H - 1 - r_) * S + S / 2)), 1, colr, -1)
    cv2.putText(img, "DEM relief (red = DEM obstacle, grey = unseen) | lidar dots: white ground, magenta obstacle",
                (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    if "img" in last:
        cam = last["img"]; cam = cv2.resize(cam, (int(cam.shape[1] * img.shape[0] / cam.shape[0]), img.shape[0]))
        img = np.hstack([img, cam])
    cv2.imwrite(f"/tmp/overlay/{k:02d}.png", img)
    np.savez(f"/tmp/overlay/{k:02d}.npz", dem=dem, lid=lid, gp=gp, res=res)
    return s


stats, t0, nxt, k = [], time.time(), time.time() + 3, 0
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() >= nxt and all(x in last for x in ("dem", "cloud", "att")):
        nxt += PERIOD
        try:
            s = snapshot(k); stats.append(s); k += 1
            print(json.dumps(s), flush=True)
        except Exception as e:
            print("snapshot error:", repr(e), flush=True)
json.dump(stats, open("/tmp/overlay/stats.json", "w"))
