#!/bin/bash
set -e

# 1. Configurar entorno gráfico
export DISPLAY=:1
export XAUTHORITY=/tmp/.xauth
touch $XAUTHORITY
xauth add $DISPLAY . $(mcookie)

# 2. Iniciar Xvfb con configuración mejorada
Xvfb $DISPLAY -screen 0 1024x768x24 +extension GLX +extension RANDR +extension RENDER -ac &> /tmp/xvfb.log &
echo "Xvfb iniciado en $DISPLAY"

# 3. Configurar VNC
mkdir -p /root/.vnc
echo "yourpassword" | vncpasswd -f > /root/.vnc/passwd
chmod 600 /root/.vnc/passwd

# 4. Iniciar x11vnc con opciones de renderizado
x11vnc -display $DISPLAY -forever -shared -rfbauth /root/.vnc/passwd \
  -noxdamage -xkb -noxrecord -noxfixes -rfbport 5900 &> /tmp/x11vnc.log &
echo "x11vnc iniciado en puerto 5900"

# 5. Configurar auto-conexión para noVNC
cat > /usr/share/novnc/index.html <<'EOF'
<html>
<head>
<script>
window.location.href = /vnc.html?autoconnect=true&password=yourpassword&view_only=false;
</script>
</head>
<body></body>
</html>
EOF

# 6. Iniciar noVNC
/usr/share/novnc/utils/launch.sh --web /usr/share/novnc \
  --vnc localhost:5900 --listen 0.0.0.0:8080 &
echo "noVNC accesible en: http://<IP_PUBLICA>:8080"

# 7. Configurar ROS
source /opt/ros/jazzy/setup.bash

# 8. Iniciar FastAPI
/opt/venv/bin/uvicorn main:app --host 0.0.0.0 --port 8000 &> /tmp/fastapi.log &

# 9. Esperar a que los servicios estén listos
sleep 2

# 10. Iniciar turtlesim con variables de entorno gráficas
echo "Iniciando turtlesim..."
exec dbus-launch ros2 run turtlesim turtlesim_node