#!/bin/bash

# Start fake X server
Xvfb :0 -screen 0 1920x1080x24 &
export DISPLAY=:0

# Start DBus session
export $(dbus-launch)

# Electron safe-mode flags for headless Raspberry Pi
export ELECTRON_DISABLE_GPU=1
export ELECTRON_ENABLE_SWIFTSHADER=1
export LIBGL_ALWAYS_SOFTWARE=1
export NO_AT_BRIDGE=1

# Start MagicMirror
cd /home/pi/MagicMirror
npm run server:logs

