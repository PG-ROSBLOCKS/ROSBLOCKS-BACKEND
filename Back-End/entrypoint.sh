#!/bin/bash
set -e  # Hace que el script termine si un comando falla

# Fuente del entorno ROS
source /opt/ros/jazzy/setup.bash

# Iniciar FastAPI en segundo plano
/opt/venv/bin/uvicorn main:app --host 0.0.0.0 --port 8000 &

# Esperar unos segundos para asegurar que FastAPI inicia correctamente
sleep 5

# Iniciar Turtlesim en primer plano
exec ros2 run turtlesim turtlesim_node
