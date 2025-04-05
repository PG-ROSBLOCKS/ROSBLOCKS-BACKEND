#!/bin/bash
set -e

# Establecer display virtual
export DISPLAY=:99
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p $XDG_RUNTIME_DIR

echo "[INFO] Iniciando Xvfb..."
Xvfb $DISPLAY -screen 0 1024x768x24 &
sleep 2
openbox > /dev/null 2>&1 &


echo "[INFO] Iniciando x11vnc..."
x11vnc -display $DISPLAY -forever -nopw -shared -rfbport 5900 -bg

echo "[INFO] Iniciando websockify + noVNC..."
mkdir -p /opt/novnc
cp -r /usr/share/novnc/* /opt/novnc/
cd /opt/novnc

# Esto sirve la interfaz web de noVNC en el puerto 8080 y la conecta al VNC local
websockify --web . --wrap-mode=ignore 8080 localhost:5900 &

echo "[INFO] Iniciando backend FastAPI..."
cd /app
exec /opt/venv/bin/uvicorn main:app --host 0.0.0.0 --port 8000


# Pequeño delay opcional
sleep 2

# Lanzar turtlesim en primer plano
exec ros2 run turtlesim turtlesim_node