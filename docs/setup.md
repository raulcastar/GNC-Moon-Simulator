# Setup

## Host: WSL2 and Docker

The stack runs in Docker on WSL2; Unreal runs on Windows. WSL reads its
settings only from your Windows profile, `%UserProfile%\.wslconfig`. Put this
there (or merge it into an existing one), then run `wsl --shutdown`:

```ini
[wsl2]
# Required. Windows and WSL share one network, so Unreal on Windows reaches
# the containers on 127.0.0.1 (and the browser reaches the dashboard).
networkingMode=mirrored

# Resource caps. Every component container runs `colcon build` on its first
# start, so a full `docker compose up` runs ~10 builds at once; an unbounded
# WSL VM can take all the host's RAM and CPU and destabilise it. Leave enough
# for Windows and Unreal (these suit a 32 GB, 22-thread machine).
memory=10GB
processors=6
swap=4GB
```

`localhostForwarding` is not needed: mirrored networking ignores it. DNS
tunnelling is on by default in current WSL.

### Keep a WSL terminal open while running

WSL2 shuts a distro down once nothing holds it open, and a systemd distro
running dockerd can still be caught by this: systemd stops `docker.service`,
every container goes down with it, and the dashboard becomes unreachable.
Keep an interactive session open the whole time the stack runs:

    wsl -d <distro>      # the distribution Docker runs in (wsl -l -v lists them)

If drops persist, run `wsl --update` then `wsl --shutdown`: premature
systemd-distro shutdowns are a known WSL bug fixed in newer builds.

On Windows, run the repo's shell scripts through WSL (`wsl -- bash <script>`).

## Build and start

```bash
./docker_build.sh          # build the base image, then all components, then start
docker compose logs -f component_stereo_depth
```

`docker_build.sh` pulls `ros:jazzy`, so it needs network access. Each component
mounts its source as a volume and builds it once, in the entrypoint all
components share (`docker/entrypoint.sh`, baked into the base image; a
component's Dockerfile only adds its dependencies and sets `GNC_PACKAGE`, the
package to launch).

## Applying code changes

To keep a restart (or a WSL/Docker reboot) from re-running ten `colcon build`s
at once, the entrypoint **skips the build when the workspace is already
built**. A plain restart is near-instant but does **not** pick up code changes.
Either rebuild inside the running container and restart it, or recreate it:

```bash
# rebuild one package in place, then restart it:
docker compose exec component_path_planner bash -lc \
  'cd /opt/ros/jazzy/src && source /opt/ros/jazzy/setup.bash && colcon build --packages-select component_path_planner'
docker compose restart component_path_planner

# or recreate the container (rebuilds its whole workspace):
docker compose up -d --force-recreate component_path_planner
```

The entrypoint also honours `FORCE_BUILD=1` in the container's environment.
New compose volumes or environment variables need `docker compose up -d
<service>`; a restart is not enough.

Restarting `component_rover_leo_raph` drops the rover's TCP link, and Unreal
only dials it when Play begins: **press Play again** after restarting it.

## ROS tools

`debug_cli` is an interactive shell with the ROS 2 tools (rviz2, rqt) on the
host display:

```bash
docker compose exec debug_cli bash
```

Only `component_path_planner` and `component_local_mapping` get the host display,
for their optional matplotlib debug windows (`enable_path_visualization` and
`debug_mode` in their `config/params.yaml`); the other nodes are headless.
