#!/usr/bin/env bash
# K.A.R.R. dev container helper (Linux, macOS, WSL).   Usage:  ./dev.sh <command> [args]
#
#   build      Build the Docker image (native arch: amd64 or arm64)
#   up         Start the container in the background
#   down       Stop and remove the container (build volume is kept)
#   shell      bash inside the container (ROS sourced, workspace at ~/ros2_ws)
#   colcon     colcon build --symlink-install [args]
#   deps       rosdep install for every package in the repo
#   verify     Regenerate karr.urdf from the SolidWorks export, build, run all checks
#   display    Build, then show K.A.R.R. in rviz2 with joint sliders (extra args go to ros2 launch)
#   glxtest    Print the OpenGL renderer the container sees
#   reset      Remove container AND build volume (clean rebuild)
#
# GUI mode is picked automatically: macOS -> vnc, Linux/WSL with $DISPLAY -> x11, else vnc.
# Override with KARR_GUI=x11|vnc|none.  vnc mode: open http://localhost:6080/vnc.html
set -euo pipefail
cd "$(dirname "$0")"

if [[ -z "${KARR_GUI:-}" ]]; then
  if [[ "$(uname -s)" == Linux && -n "${DISPLAY:-}" ]]; then KARR_GUI=x11; else KARR_GUI=vnc; fi
fi
files=(-f docker/compose.yaml)
[[ "$KARR_GUI" != none ]] && files+=(-f "docker/gui-$KARR_GUI.yaml")
if [[ "$(uname -s)" == Linux ]]; then export USER_UID="$(id -u)" USER_GID="$(id -g)"; fi

dc() { docker compose "${files[@]}" "$@"; }
up() {
  [[ "$KARR_GUI" == x11 ]] && command -v xhost >/dev/null && xhost +local: >/dev/null
  dc up -d
  [[ "$KARR_GUI" == vnc ]] && echo "GUI: http://localhost:6080/vnc.html?autoconnect=1&resize=remote"
  return 0
}
in_ws() { dc exec ros ws-env bash -c "$1"; }

cmd="${1:-help}"; shift || true
case "$cmd" in
  build)   dc build ;;
  up)      up ;;
  down)    dc down ;;
  shell)   up; dc exec ros bash ;;
  colcon)  up; in_ws "colcon build --symlink-install $*" ;;
  deps)    up; in_ws "sudo apt-get update && rosdep install --from-paths src --ignore-src -y" ;;
  verify)  up; dc exec ros bash /home/ubuntu/ros2_ws/src/karr_robot/urdf_prep/tools/verify.sh ;;
  display) up; in_ws "colcon build --symlink-install && source install/setup.bash && ros2 launch karr_description display.launch.py $*" ;;
  glxtest) up; in_ws "glxinfo -B | grep -i renderer" ;;
  reset)   dc down -v ;;
  *)       sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//' ;;
esac
