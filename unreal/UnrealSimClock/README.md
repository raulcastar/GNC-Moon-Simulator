# UnrealSimClock — the simulation time, streamed to ROS as /clock

**The simulation time** is defined in `GncSimTime.h`, a small header of which
every GNC sensor plugin (UnrealStereoCam, UnrealLidarCam, UnrealEventCam,
UnrealSPADCam) and this plugin carry an **identical copy** in their `Private`
folder, so no plugin depends on another: `Anchor + World->GetTimeSeconds()`, in
nanoseconds since the Unix epoch. It follows Unreal's game time (stops when Play
is paused, follows time dilation). The anchor is the wall clock minus the world
time when the first plugin asks in a Play session, stored as a tag on the
world's World Settings so all plugins share it; a new Play session takes a new,
later anchor, so the clock does not run backwards. Outside Play it is the wall
clock. Change the copies together (they are checked identical by `md5sum
unreal/*/Source/*/Private/GncSimTime.h`).

**This plugin** only publishes that time. A world subsystem, so nothing to add
to a Blueprint or level: in every Play session it sends it to
`component_sim_clock` over TCP every world tick, one 24-byte
  record each, which that node publishes on `/clock`. The plugin dials the node
  when Play begins and redials every 2 s while it is not there; connecting and
  sending never block the game thread (a record the socket cannot take yet is
  finished on the next tick):

```
offset size  field
0      4     magic 'SCLK'
4      1     version = 1
5      3     reserved
8      8     sim time, ns since the Unix epoch (u64, little-endian)
16     8     frame (u64): GFrameCounter
```

Console variables (read when Play begins): `simclock.Host` (127.0.0.1),
`simclock.Port` (3008), `simclock.MaxRateHz` (200: at most that many records a
second; one per tick below it).

Install: copy this folder into `<Project>/Plugins/` with the other sensor
plugins and build (the editor offers to, or `Build.bat UnrealEditor Win64
Development -Project=<.uproject>` with the editor closed).
