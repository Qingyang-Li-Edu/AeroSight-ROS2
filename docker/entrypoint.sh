#!/bin/bash
# Virtual X display + VNC + noVNC (browser on :6080), then the G1 launcher.
export DISPLAY=:1 LIBGL_ALWAYS_SOFTWARE=1
Xvfb :1 -screen 0 1600x900x24 &
sleep 1
fluxbox >/dev/null 2>&1 &
x11vnc -display :1 -forever -shared -nopw -quiet &
websockify --web=/usr/share/novnc 6080 localhost:5900 >/dev/null 2>&1 &
exec "$(dirname "$0")/../scripts/start.sh" "$@"
