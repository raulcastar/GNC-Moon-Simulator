"""
Enumerate the cameras UnrealCV actually exposes, and check whether any pair of
them forms a usable stereo rig.

Run this with the stereo node stopped — UnrealCV serves one client at a time,
so component_stereo_depth will be holding the socket.

    docker compose stop component_stereo_depth
    python utils/list_unreal_cameras.py

Prints, for each camera id, its location, rotation, FOV and resolution. Then for
each ordered pair it reports whether the rotations match and whether the offset
is purely lateral, which is what SGBM needs, and what baseline you would get.
"""

import math
import os
import sys
import time

import numpy as np
from unrealcv import Client
from unrealcv.util import read_png

UU_PER_METRE = 1000.0    # keep in sync with component_stereo_depth/config/params.yaml


def unreal_axes(pitch_deg, yaw_deg, roll_deg):
    """Forward / right / up for an Unreal FRotator (mirrors FRotationMatrix)."""
    p, y, r = map(math.radians, (pitch_deg, yaw_deg, roll_deg))
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    cr, sr = math.cos(r), math.sin(r)
    forward = np.array([cp * cy, cp * sy, sp])
    right = np.array([sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp])
    up = np.array([-(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp])
    return forward, right, up


def ask(client, cmd, retries=3):
    """Request a command, tolerating dropped replies and reporting rejections."""
    for _ in range(retries):
        res = client.request(cmd)
        if res is None:
            continue
        if isinstance(res, bytes):
            res = res.decode('utf-8', 'replace')
        if res.lower().startswith('error'):
            return None
        return res
    return None


def floats(res):
    try:
        return np.array([float(v) for v in res.split()])
    except (AttributeError, ValueError):
        return None


def main():
    ip = "127.0.0.1"
    if os.path.exists("ip.txt"):
        with open("ip.txt") as f:
            ip = f.readline().strip()

    client = Client((ip, 9000), 'inet')
    for _ in range(10):
        if client.isconnected():
            break
        client.connect()
        time.sleep(1)
    if not client.isconnected():
        sys.exit("Could not connect. Is another component holding the socket?")

    try:
        raw = ask(client, 'vget /cameras')
        if raw is None:
            sys.exit("Server does not support 'vget /cameras'.")
        ids = list(range(len(raw.split())))
        print(f"vget /cameras -> {raw!r}")
        print(f"{len(ids)} camera(s), valid ids {ids}\n")

        cams = {}
        for cid in ids:
            loc = floats(ask(client, f'vget /camera/{cid}/location'))
            rot = floats(ask(client, f'vget /camera/{cid}/rotation'))
            fov = ask(client, f'vget /camera/{cid}/fov')
            png = ask(client, f'vget /camera/{cid}/lit png')
            try:
                h, w = read_png(png).shape[:2]
                res = f'{w}x{h}'
            except Exception:
                res = '?'
            cams[cid] = (loc, rot)
            loc_s = np.array2string(loc, precision=1) if loc is not None else 'n/a'
            rot_s = np.array2string(rot, precision=1) if rot is not None else 'n/a'
            print(f"  camera {cid}: loc(UU)={loc_s}  rot(p,y,r)={rot_s}  "
                  f"fov={fov or 'n/a'}  res={res}")

        if len(ids) < 2:
            print("\nOnly one camera. A stereo pair needs two cameras with the "
                  "same rotation, offset along the left camera's right axis.")
            return

        print("\nStereo pair candidates (left -> right):")
        for a in ids:
            for b in ids:
                if a == b or cams[a][0] is None or cams[b][0] is None:
                    continue
                loc_a, rot_a = cams[a]
                loc_b, _ = cams[b]
                rot_err = float(np.abs(cams[b][1] - rot_a).max())
                _, right_vec, _ = unreal_axes(*rot_a)
                offset = loc_b - loc_a
                lateral = float(np.dot(offset, right_vec))
                residual = float(np.linalg.norm(offset - lateral * right_vec))

                if abs(lateral) < 1e-6:
                    verdict = "no lateral offset"
                elif rot_err > 1e-3:
                    verdict = f"rotations differ by {rot_err:.2f} deg -> not rectified"
                elif residual > 0.01 * abs(lateral):
                    verdict = f"{residual:.1f} UU of the offset is vertical/forward"
                else:
                    verdict = "USABLE"
                print(f"  {a} -> {b}: baseline {abs(lateral):7.2f} UU "
                      f"= {abs(lateral) / UU_PER_METRE * 100:6.2f} cm   {verdict}")

        print("\nSet left_camera_id / right_camera_id in "
              "component_stereo_depth/config/params.yaml to a USABLE pair.")
    finally:
        client.disconnect()


if __name__ == '__main__':
    main()
