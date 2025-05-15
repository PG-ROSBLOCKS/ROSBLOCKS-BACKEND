#!/bin/bash
set -e

# Establecer display virtual
export DISPLAY=:99
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p $XDG_RUNTIME_DIR
chmod 700 "$XDG_RUNTIME_DIR"


echo "[INFO] Iniciando Xvfb..."
Xvfb $DISPLAY -screen 0 500x500x24 &
sleep 2
fluxbox > /dev/null 2>&1 &


echo "[INFO] Iniciando x11vnc..."
x11vnc -display :99 -forever -shared -nopw -rfbport 5900 -bg -quiet -noxdamage -repeat -nowf

echo "[INFO] Iniciando websockify + noVNC..."

# Esto sirve la interfaz web de noVNC en el puerto 8080 y la conecta al VNC local
websockify --web=/opt/novnc/ --wrap-mode=ignore 8080 localhost:5900 &

echo "[INFO] Iniciando backend FastAPI..."
cd /
exec /opt/venv/bin/uvicorn app.main:app --host 0.0.0.0 --port 8000 &


# Pequeño delay opcional
sleep 2


# Lanzar turtlesim en primer plano
source /opt/ros/jazzy/setup.bash
exec ros2 run turtlesim turtlesim_node