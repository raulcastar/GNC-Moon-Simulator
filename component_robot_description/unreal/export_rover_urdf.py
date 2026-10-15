"""
Writes the simulated rover's URDF from its Unreal Blueprint. Run inside the
Unreal Editor (Tools > Execute Python Script, or remote execution), with or
without Play running:

    exec(open(r"<repo>/component_robot_description/unreal/export_rover_urdf.py").read())

It reads the placed rover (class Leo_Raph* or Yutu_Rover*) and writes
component_robot_description/urdf/<rover>.urdf (leo_raph.urdf, yutu.urdf) next to
this folder. Regenerate it whenever a sensor or wheel moves in the Blueprint.
Positions and orientations are measured in base_link's axes, so it does not
matter how the body mesh is turned in the Blueprint.

Frames (ROS conventions, REP-103/105):
    base_link        the Blueprint's base_link component, which the rover's
                     telemetry reports (odometry): centred between the wheels
                     at axle height. Axes: x forward, y left, z up (the
                     Blueprint's +Y, +X and +Z: Unreal is left-handed, so two
                     axes swap).
    base_footprint   on the ground under base_link (the wheels' radius below their centres)
    wheel_*_link     wheel centres (fixed: no joint states are published)
    <camera>_link    camera body frames: x along the view, z up
    <camera>         the optical frame the images are stamped with: z along the
                     view, x right, y down (the stereo node's frame names)
    lidar            the lidar's sensor frame (x forward, y left, z up), at the
                     component it follows (FollowComponentName)
"""

import math
import os

import unreal

# Unreal component -> ROS frame name (the frame_id the nodes stamp with).
CAMERAS = {
    "Navcam_Left": "navcam_left",
    "Navcam_Center": "navcam_central",
    "Navcam_Right": "navcam_right",
    "RearCam": "rear_cam",
    "NavCam_EventCam": "event_camera",
    "Navcam_SPADCam": "spad",
}
WHEELS = {                               # the middle pair only on six-wheeled rovers (Yutu)
    "Raph_Rover_FL_Wheel": "wheel_fl_link",
    "Raph_Rover_FR_Wheel": "wheel_fr_link",
    "Raph_Rover_ML_Wheel": "wheel_ml_link",
    "Raph_Rover_MR_Wheel": "wheel_mr_link",
    "Raph_Rover_BL_Wheel": "wheel_rl_link",
    "Raph_Rover_BR_Wheel": "wheel_rr_link",
}
# Rover Blueprint class prefix -> URDF file and robot name. Yutu_Rover is built from a
# copy of Leo_Raph, so both share the component names above.
ROVERS = {"Leo_Raph": "leo_raph", "Yutu_Rover": "yutu"}
BODY = "Raph_Rover_Body"
ODOMETRY_POINT = "base_link"            # the component the telemetry reports (odometry)
LIDAR_CLASS = "UnrealLidarCam"

try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:                        # exec() without a file: use OUTPUT_DIR if set
    HERE = globals().get("OUTPUT_DIR", os.getcwd())
URDF_DIR = os.path.join(os.path.dirname(HERE), "urdf")


def find_rover():
    world = unreal.EditorLevelLibrary.get_game_world()
    if world is not None:
        actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Pawn)
    else:
        actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem).get_all_level_actors()
    rovers = [a for a in actors if a.get_class().get_name().startswith(tuple(ROVERS))]
    if not rovers:
        raise RuntimeError(f"No rover ({', '.join(ROVERS)}) in the level")
    return rovers[0]


def ros_vector(v, scale=1.0):
    """An Unreal vector in base_link's axes (X forward, Y right, Z up) -> ROS (x forward, y left, z up)."""
    return (v.x * scale, -v.y * scale, v.z * scale)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def rpy_from_axes(x, y, z):
    """Roll, pitch, yaw (rad, fixed XYZ as URDF uses) of the frame whose axes are x, y, z."""
    # Rotation matrix columns are the frame's axes.
    r00, r10, r20 = x
    r21, r22 = y[2], z[2]
    yaw = math.atan2(r10, r00)
    pitch = math.asin(max(-1.0, min(1.0, -r20)))
    roll = math.atan2(r21, r22)
    return roll, pitch, yaw


def relative_pose(base_transform, component):
    """(xyz, rpy) of a component in base_link: position in metres, x along its view, z up."""
    t = component.get_world_transform()
    loc = base_transform.inverse_transform_location(t.translation)
    rot = base_transform.inverse_transform_rotation(t.rotation.rotator())
    xyz = ros_vector(loc, 0.01)
    x = ros_vector(unreal.MathLibrary.get_forward_vector(rot))
    z = ros_vector(unreal.MathLibrary.get_up_vector(rot))
    y = cross(z, x)
    return xyz, rpy_from_axes(x, y, z)


def joint(name, parent, child, xyz, rpy):
    f = lambda v, digits: " ".join(f"{c:.{digits}f}" for c in v)
    return (f'  <joint name="{name}" type="fixed">\n'
            f'    <origin xyz="{f(xyz, 4)}" rpy="{f(rpy, 6)}"/>\n'
            f'    <parent link="{parent}"/>\n    <child link="{child}"/>\n  </joint>\n')


def export():
    rover = find_rover()
    key = next(v for k, v in ROVERS.items() if rover.get_class().get_name().startswith(k))
    output = os.path.join(URDF_DIR, f"{key}.urdf")
    comps = {c.get_name(): c for c in rover.get_components_by_class(unreal.SceneComponent)}
    body = comps[BODY]
    # Every frame is measured in base_link's axes (X forward, Z up on every rover),
    # whatever way the body mesh is turned.
    bt = comps[ODOMETRY_POINT].get_world_transform()

    out = ['<?xml version="1.0"?>\n',
           '<!-- GENERATED by component_robot_description/unreal/export_rover_urdf.py from the\n'
           f'     Unreal Blueprint {rover.get_class().get_name()}: do not edit by hand, regenerate. -->\n',
           f'<robot name="{key}_unreal">\n']

    # Body and wheels (simple shapes from the meshes, masses from the physics bodies).
    # The body's box, turned into base_link's axes.
    ext = body.get_editor_property("static_mesh").get_bounds().box_extent
    s = body.get_world_scale()
    rel = bt.inverse_transform_rotation(body.get_world_transform().rotation.rotator())
    axes = [unreal.MathLibrary.get_forward_vector(rel), unreal.MathLibrary.get_right_vector(rel),
            unreal.MathLibrary.get_up_vector(rel)]
    half = [ext.x * s.x, ext.y * s.y, ext.z * s.z]
    span = [2 * sum(abs((a.x, a.y, a.z)[i]) * half[j] for j, a in enumerate(axes)) for i in range(3)]
    size = ros_vector(unreal.Vector(*span), 0.01)
    mass = body.get_mass() if body.is_simulating_physics() else body.get_editor_property("body_instance").get_editor_property("mass_in_kg_override")
    # No <inertial> on the root link: KDL (robot_state_publisher) refuses one.
    out.append(f'  <!-- body mass {mass:.1f} kg (Unreal physics) -->\n  <link name="base_link">\n'
               f'    <visual><geometry><box size="{abs(size[0]):.3f} {abs(size[1]):.3f} {abs(size[2]):.3f}"/></geometry></visual>\n'
               '  </link>\n')

    wheel_z, radius, width = [], 0.0, 0.0
    for comp_name, link in WHEELS.items():
        if comp_name not in comps:
            continue
        w = comps[comp_name]
        xyz, _ = relative_pose(bt, w)
        wheel_z.append(xyz[2])
        e = w.get_editor_property("static_mesh").get_bounds().box_extent
        sc = w.get_world_scale()
        dims = sorted((abs(e.x * sc.x), abs(e.y * sc.y), abs(e.z * sc.z)))
        radius, width = dims[2] * 0.01, 2 * dims[0] * 0.01
        out.append(f'  <link name="{link}">\n    <visual><origin rpy="1.5708 0 0"/>'
                   f'<geometry><cylinder radius="{radius:.4f}" length="{width:.4f}"/></geometry></visual>\n  </link>\n')
        out.append(joint(f"{link}_joint", "base_link", link, xyz, (0.0, 0.0, 0.0)))

    height = -sum(wheel_z) / len(wheel_z) + radius
    out.append('  <link name="base_footprint"/>\n')
    out.append(joint("base_footprint_joint", "base_link", "base_footprint", (0.0, 0.0, -height), (0.0, 0.0, 0.0)))

    for comp_name, frame in CAMERAS.items():
        if comp_name not in comps:
            unreal.log_warning(f"[urdf] camera component {comp_name} not found: {frame} skipped")
            continue
        xyz, rpy = relative_pose(bt, comps[comp_name])
        out.append(f'  <link name="{frame}_link"/>\n  <link name="{frame}"/>\n')
        out.append(joint(f"{frame}_joint", "base_link", f"{frame}_link", xyz, rpy))
        out.append(joint(f"{frame}_optical_joint", f"{frame}_link", frame, (0.0, 0.0, 0.0),
                         (-math.pi / 2, 0.0, -math.pi / 2)))

    lidars = [c for c in rover.get_components_by_class(unreal.ActorComponent) if c.get_class().get_name() == LIDAR_CLASS]
    if lidars:
        follow = str(lidars[0].get_editor_property("follow_component_name"))
        mount = comps.get(follow, lidars[0])
        xyz, rpy = relative_pose(bt, mount)
        out.append('  <link name="lidar"/>\n')
        out.append(joint("lidar_joint", "base_link", "lidar", xyz, rpy))

    out.append('</robot>\n')
    os.makedirs(URDF_DIR, exist_ok=True)
    with open(output, "w", newline="\n") as handle:
        handle.write("".join(out))
    unreal.log(f"[urdf] wrote {output}: base_link {height:.3f} m above the ground, wheels r {radius:.3f} m")
    print(f"URDF written: {output}")


export()
