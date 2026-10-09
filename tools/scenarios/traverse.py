"""
Traverse: send the rover a goal and log the drive (pose, attitude, commands,
mission state, lidar obstacles ahead) until it arrives, the planner gives up,
the lidar sees an obstacle close ahead, the rover is stuck, or time runs out.
It always cancels the goal and stops the rover at the end.

Runs inside the component_path_planner container during Play:
    tools/scenarios/run.sh traverse <goal_x> <goal_y> [seconds]
Writes /tmp/traverse_log.json (one row every 0.5 s); run.sh copies it to
tools/scenarios/results/.
"""
import json, math, sys, time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Empty

gx, gy = float(sys.argv[1]), float(sys.argv[2])
DURATION = float(sys.argv[3]) if len(sys.argv) > 3 else 300.0
# Lidar mounting (pitched down, as in component_lidar), corridor half width, obstacle height.
LIDAR_PITCH, HALF_W, MIN_H = math.radians(-20.0), 0.5, 0.25
pose, st, last_cmd, lidar = {}, {"s": ""}, {"v": 0.0, "w": 0.0}, {"ahead": None, "returns": 0, "n": 0}
DT = np.dtype({"names": ["x", "y", "z", "i"], "formats": ["<f4"] * 4, "offsets": [0, 4, 8, 12], "itemsize": 20})


def on_odom(m):
    q = m.pose.pose.orientation
    pose.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y, z=m.pose.pose.position.z,
                yaw=math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
                pitch=math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x)))),
                roll=math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))


def on_cloud(m):
    a = np.frombuffer(bytes(m.data), DT)
    ok = np.isfinite(a["x"])
    x, y, z = a["x"][ok], a["y"][ok], a["z"][ok]
    # Sensor -> rover frame (lidar pitched down), then height above a ground
    # plane fitted to the near field (absorbs rover tilt and terrain slope).
    p = -LIDAR_PITCH
    xl, zl = x * math.cos(p) + z * math.sin(p), -x * math.sin(p) + z * math.cos(p)
    near = (xl < 8) & (np.abs(y) < 3)
    keep = near.copy()
    for _ in range(4):
        A = np.c_[np.ones(keep.sum()), xl[keep], y[keep]]
        coef = np.linalg.lstsq(A, zl[keep], rcond=None)[0]
        h = zl - (coef[0] + coef[1] * xl + coef[2] * y)
        keep = near & (h < 0.08)
    lidar["plane_h"] = float(-coef[0])           # Lidar height above the fitted ground
    hit = (np.abs(y) < HALF_W) & (xl > 0.3) & (xl < 8) & (h > MIN_H)
    lidar["returns"], lidar["n"] = int(ok.sum()), lidar["n"] + 1
    if hit.sum() >= 15:
        xs = np.sort(xl[hit]); lidar["ahead"] = float(xs[min(14, len(xs) - 1)])
        lidar["ahead_h"] = float(np.percentile(h[hit], 95))
    else:
        lidar["ahead"], lidar["ahead_h"] = None, None


def on_cmd(m):
    last_cmd.update(v=m.linear.x, w=m.angular.z)


rclpy.init()
n = rclpy.create_node("traverse_logger")
n.create_subscription(Odometry, "/odom", on_odom, 10)
n.create_subscription(PointCloud2, "/lidar/points", on_cloud, 2)
n.create_subscription(Twist, "/cmd_vel", on_cmd, 10)
n.create_subscription(String, "/mission_status/state", lambda m: st.__setitem__("s", m.data), 10)
goal_pub = n.create_publisher(PoseStamped, "/goal_pose", 10)
cancel = n.create_publisher(Empty, "/cancel_goal", 10)
cmd = n.create_publisher(Twist, "/cmd_vel", 10)
log = []


def spin(sec):
    end = time.time() + sec
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)


def stop():
    for _ in range(5):
        cmd.publish(Twist()); spin(0.1)


try:
    spin(2.0)
    print("start", {k: round(v, 2) for k, v in pose.items()}, flush=True)
    g = PoseStamped(); g.header.frame_id = "map"; g.pose.position.x = gx; g.pose.position.y = gy
    goal_pub.publish(g)
    t0, last_s, hist, why = time.time(), None, [], "timeout"
    while time.time() - t0 < DURATION:
        spin(0.5)
        t = time.time() - t0
        row = dict(t=round(t, 1), **{k: round(v, 3) for k, v in pose.items()}, v=round(last_cmd["v"], 3),
                   w=round(last_cmd["w"], 3), ahead=lidar["ahead"], ahead_h=lidar.get("ahead_h"), plane_h=lidar.get("plane_h"), ret=lidar["returns"])
        log.append(row)
        s = st["s"]
        if s != last_s:
            print(f"{t:5.1f}s state: {s[:140]}", flush=True); last_s = s
        if int(t * 2) % 10 == 0:
            print(f"{t:5.1f}s x {pose['x']:.2f} y {pose['y']:.2f} yaw {math.degrees(pose['yaw']):6.1f} "
                  f"pitch {math.degrees(pose['pitch']):5.1f} roll {math.degrees(pose['roll']):5.1f} | cmd v {last_cmd['v']:.2f} "
                  f"w {last_cmd['w']:.2f} | lidar obstacle ahead: "
                  + f"[ground {lidar.get('plane_h', 0):.2f} m below] " + (f"{lidar['ahead']:.1f} m ({lidar['ahead_h']:.2f} m tall)" if lidar["ahead"] else "none"), flush=True)
        if lidar["ahead"] is not None and lidar["ahead"] < 1.5 and t > 10:
            why = "lidar obstacle %.1f m ahead (%.2f m tall): stopped before it" % (lidar["ahead"], lidar["ahead_h"]); break
        hist.append((t, pose["x"], pose["y"]))
        old = [h for h in hist if t - h[0] > 25]
        if old and t > 40 and math.hypot(pose["x"] - old[-1][1], pose["y"] - old[-1][2]) < 0.3:
            why = "stuck (moved < 0.3 m in 25 s)"; break
        if t > 5 and any(k in s.lower() for k in ('"reached', "aborted", "failed", "cancelled", '"idle')):
            why = "planner: " + s[:100]; break
    print("end:", why, "| dist to goal %.2f m" % math.hypot(pose["x"] - gx, pose["y"] - gy), flush=True)
finally:
    cancel.publish(Empty()); stop()
    json.dump(log, open("/tmp/traverse_log.json", "w"))
