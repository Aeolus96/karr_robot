# karr_robot

Everything for the K.A.R.R. robot (LTU): ROS 2 Jazzy packages, the URDF pipeline from SolidWorks, and a dev container.
One clone sets it all up.

```
karr_robot/
├── karr_description/   ROS 2 package: URDF, meshes, launch, rviz          (built by colcon)
├── urdf_prep/          SolidWorks export + converter tools (Windows side)  (COLCON_IGNORE)
├── docker/             Dev container (ROS 2 Jazzy, Ubuntu 24.04, amd64 + arm64)
├── dev.sh              Container helper for Linux / macOS / WSL
└── dev.ps1             Container helper for Windows PowerShell
```

New ROS 2 packages go at the top level next to `karr_description/`. Other repos (drivers etc.) can be added as git
submodules anywhere in the tree; colcon finds packages recursively.

## Use as a ROS 2 workspace (robot / Ubuntu 24.04)

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --recurse-submodules <repo-url> karr_robot
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
ros2 launch karr_description display.launch.py
```

## Use the dev container (any laptop)

Needs Docker (Docker Desktop on Windows/macOS). The image builds natively for the host: x86_64 laptops, Apple Silicon
and Jetson all work from the same Dockerfile. Inside the container the repo is mounted at `~/ros2_ws/src/karr_robot`,
the same layout as on the robot; `build/ install/ log/` live in a Docker volume.

```bash
./dev.sh build      # first time, ~5-10 min
./dev.sh display    # rviz2 + joint sliders
./dev.sh shell      # bash with ROS sourced
```

Windows: same commands with `.\dev.ps1` (run `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` once if blocked).

| Command | What it does |
|---|---|
| `build` | Build the image |
| `up` / `down` | Start / stop the container (`down` keeps the build volume) |
| `shell` | bash in the container |
| `colcon [args]` | `colcon build --symlink-install` |
| `deps` | `rosdep install` for every package in the repo |
| `display [args]` | Build, then `ros2 launch karr_description display.launch.py` |
| `verify` | Regenerate the URDF from the SolidWorks export and run all checks (see `urdf_prep/`) |
| `glxtest` | Show which OpenGL renderer the container uses |
| `reset` | Remove container and build volume |

### GUI

Picked automatically; override with `KARR_GUI=x11|wslg|vnc|none`.

| Host | Default | How it shows |
|---|---|---|
| Ubuntu (desktop session) | `x11` | Native windows via the host X server. Also uses host networking, so ROS 2 topics reach the robot on the LAN. |
| Windows 11 (PowerShell) | `wslg` | Native windows through WSLg. Needs Docker Desktop with the WSL 2 engine. |
| macOS, or anything headless | `vnc` | Desktop in the browser: <http://localhost:6080/vnc.html?autoconnect=1&resize=remote> |

Rendering is software OpenGL (`LIBGL_ALWAYS_SOFTWARE=1`) by default, which is reliable for rviz. To try the GPU on
x11/wslg: `LIBGL_ALWAYS_SOFTWARE=0 ./dev.sh up`, then `./dev.sh glxtest`.

Only the `x11` mode (Linux) uses host networking. On Docker Desktop (Windows/macOS) the container is on a bridge network,
so ROS 2 discovery with the real robot won't work from there; it's for working with the packages, rviz and sim.

Windows troubleshooting: `could not connect to display` → `wsl --shutdown`, open any WSL distro once, restart Docker
Desktop. `QStandardPaths: wrong permissions on runtime directory` is harmless with WSLg.
