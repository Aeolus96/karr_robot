#!/usr/bin/env bash
# Container entrypoint. With KARR_VNC=1, starts a virtual X display and serves it in the browser via noVNC
# (http://localhost:6080/vnc.html). Otherwise the GUI goes to the host's X server / WSLg.
set -e
if [[ "${KARR_VNC:-0}" == "1" ]]; then
  export DISPLAY=:1
  rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
  Xvnc :1 -geometry "${VNC_GEOMETRY:-1600x900}" -depth 24 -SecurityTypes None -localhost yes \
       -AlwaysShared >/tmp/xvnc.log 2>&1 &
  for _ in $(seq 50); do [[ -S /tmp/.X11-unix/X1 ]] && break; sleep 0.1; done
  fluxbox >/tmp/fluxbox.log 2>&1 &
  websockify --web /usr/share/novnc 6080 localhost:5901 >/tmp/novnc.log 2>&1 &
fi
exec "$@"
