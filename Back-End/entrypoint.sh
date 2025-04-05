#!/bin/bash
set -e

export DISPLAY=:99
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p $XDG_RUNTIME_DIR

# Iniciar Xvfb
Xvfb $DISPLAY -screen 0 1024x768x24 &
sleep 1

# Verificar que el display está listo
for i in {1..10}; do
    if xdpyinfo -display $DISPLAY > /dev/null 2>&1; then
        echo "Xvfb está listo"
        break
    fi
    echo "Esperando a Xvfb..."
    sleep 1
done

# Iniciar fluxbox (en segundo plano)
fluxbox > /dev/null 2>&1 &

# Iniciar x11vnc
x11vnc -display $DISPLAY -forever -nopw -shared -rfbport 5900 -bg

# Esperar a que x11vnc abra el puerto
for i in {1..10}; do
    if nc -z localhost 5900; then
        echo "x11vnc está listo"
        break
    fi
    echo "Esperando a que x11vnc inicie..."
    sleep 1
done

# Iniciar websockify para noVNC
websockify --web=/usr/share/novnc/ --wrap-mode=ignore 8080 localhost:5900 &

# Fuente de ROS y backend
source /opt/ros/jazzy/setup.bash
cd /app
/opt/venv/bin/uvicorn main:app --host 0.0.0.0 --port 8000 &

# Lanzar turtlesim
exec ros2 run turtlesim turtlesim_node
