#!/usr/bin/env bash
# Sourced from .bashrc, or used as a wrapper:  ws-env <command...>
source /opt/ros/jazzy/setup.bash
WS="$HOME/ros2_ws"
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
# noVNC mode: GUI apps go to the virtual display started by the entrypoint
[[ "${KARR_VNC:-0}" == "1" ]] && export DISPLAY=:1
export RCUTILS_COLORIZED_OUTPUT=1
# Only exec when run as a command, not when sourced
if [[ "${BASH_SOURCE[0]}" == "$0" && $# -gt 0 ]]; then
  exec "$@"
fi
