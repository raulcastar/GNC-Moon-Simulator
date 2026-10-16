# AGENTS.md: GNC Moon Simulator

Guidance for AI coding agents working in this repository. The README has the
overview and links to `docs/`; this file covers what an agent needs to work safely.

## What this is

A ROS 2 Jazzy GNC stack for a simulated lunar rover. Each `component_*` package
is a ROS node in its own Docker container (WSL2 on Windows, Zenoh middleware).
Unreal Engine 5.7 runs the simulation on the Windows host. Its sensor plugins
(in the Unreal project's `Plugins/`) and the rover Blueprint dial the containers on `127.0.0.1`
when Play starts. The terrain, rocks, masks and sun come from Terrain Studio,
in the sibling repo `Unreal-Moon-Terrain-Generation`.

The Unreal project (`Moon_Simulator`) is its own git repository, outside this
one; its location differs per machine. Ask the user, or read it from the
running editor with `unreal.Paths.project_dir()` (Terrain Studio's
`unreal_status` tool shows the open level).

## Layout

| Path | What |
|---|---|
| `component_<name>/` | One ROS package each: `src/`, `config/params.yaml` (+ `params_<rover>.yaml`), `launch/`, `test/`, `Dockerfile` |
| `mission_control_dashboard/` | Web mission control (Flask + SocketIO, ports 5000-5007) and the JSON API (`src/api.py`, `/api/` on 5001) |
| `mcp/gnc_mcp.py` | MCP server for agents, a thin client of the JSON API |
| `unreal/python/` | Editor Python (viewport cameras), copied to the Unreal project's `Content/Python`. The sensor plugins (C++) live only in the Unreal project's `Plugins/` |
| `tools/scenarios/` | Checks that need Unreal in Play |
| `docker/`, `docker-compose.yml`, `.env` | Base image and entrypoint, services, `GNC_ROVER` (`leo_raph` or `yutu`) |

## Build, run, test

```bash
docker compose up -d                       # start the stack (each container builds once)
./run_tests.sh                             # every package's unit tests, in its container
./run_tests.sh component_path_planner      # one package
py -m pytest mcp                           # the MCP server's tests (host, Windows)
```

- **Containers don't pick up code changes on restart.** The entrypoint skips
  the build once a workspace is built. After editing a package, rebuild it
  inside its container, then restart it:
  ```bash
  docker compose exec -T <pkg> bash -lc "cd /opt/ros/jazzy/src && source /opt/ros/jazzy/setup.bash && colcon build --packages-select <pkg>"
  docker compose restart <pkg>
  ```
- On Windows, run shell scripts through WSL: `wsl -- bash <script>` (add
  `-d <distro>` if Docker runs in a distribution that is not the default).
  From Git Bash, prefix `MSYS_NO_PATHCONV=1` so paths are not rewritten.
- New compose volumes or environment variables need
  `docker compose up -d <service>`; a restart is not enough.

## Rules that prevent real damage

These come from actual incidents.

- **Restarting `component_rover_leo_raph` needs Play to be restarted.** Unreal
  only dials the rover's TCP link when Play begins. Don't restart the rover
  node while someone is driving.
- **Never set properties on the rover's components during Play.** Physics
  constraints break (the wheels came off). Read-only queries are fine.
- **Never delete, duplicate or rename an Unreal level by script.** Deleting a
  loaded level, or `duplicate_asset` plus `load_level`, crashed the editor
  ("World Memory Leaks"). Ask the user to use Save Level As, and back up first.
- **No headless-browser (WebGL) tests on the dashboard's 3D pages while Unreal
  is in Play.** Doing so crashed Unreal (GPU device removed).
- **The rover settles for about 10 s after Play starts.** Wait until its pose
  is still before driving or measuring.
- **Lifecycle restarts:** every node's launch file activates it automatically
  after configure. Don't send your own `activate` after a `configure`: the
  second activate raises inside the node and kills its executor. Use
  `restart_node` / `HealthMixin._restart_node_impl`, which waits for the
  automatic one.
- **Tests must not reach Unreal or ROS:** stub the senders and nodes, as the
  existing tests do.
- **Don't commit or push unless the user asks.**

## Conventions

- Parameters live in `config/params.yaml`. Per-rover overrides go in
  `config/params_<rover>.yaml`, loaded after it (`GNC_ROVER`). The dashboard's
  ROVER selector applies the same files live (`mission_control_dashboard/src/rovers.py`).
- Code style: comments explain *why* in full sentences, and match the
  surrounding density. Every node is a lifecycle node, with
  `use_sim_time: true` (`/clock` from Unreal).
- When you change behaviour, update the page in `docs/` that documents it (and the README if it changes the overview).

## Using the simulator as an agent (MCP)

`mcp/gnc_mcp.py` exposes the running stack as tools:

- **Reading:** `get_status`, `get_log`, `get_camera_image`, `get_rover`.
- **Driving:** `send_goal` + `wait_for_mission`, `drive`, `stop`.
- **Settings:** `set_headlights`, `set_obstacle_avoidance`,
  `set_viewport_camera`, `switch_rover`.
- **Maintenance:** `restart_node`, `record_bag`.

`.mcp.json` registers it for Claude Code. Setup:
`py -m pip install -r mcp/requirements.txt`. It needs the stack up, and for
driving, Unreal in Play. Starting and stopping Play, and building terrain, are
done by the Terrain Studio server (`Unreal-Moon-Terrain-Generation/mcp`).
The same API is available to plain scripts over HTTP
(`curl 127.0.0.1:5001/api/status`).
