FROM ros:jazzy

# Instalar dependencias necesarias
RUN apt update && apt install -y --no-install-recomends\
    python3-pip python3-dev python3-venv tmux \
    ros-jazzy-turtlesim ros-dev-tools \
    x11vnc novnc qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools \
    xfce4 xfce4-goodies dbus-x11 xvfb && \
    rm -rf /var/lib/apt/lists/*

RUN rm -rf /usr/share/novnc && \
    git clone https://github.com/novnc/noVNC.git /usr/share/novnc && \
    cd /usr/share/novnc && git checkout v1.2.0
RUN apt-get update && \
    apt-get install -y --no-install-recommends tightvncserver && \
    rm -rf /var/lib/apt/lists/*

# Crear entorno virtual para FastAPI
RUN python3 -m venv /opt/venv && \
    /opt/venv/bin/pip install --upgrade pip

ENV PATH="/opt/venv/bin:$PATH"
ENV DISPLAY=:1 
ENV QT_QPA_PLATFORM=offscreen 

# Instalar FastAPI y Uvicorn
COPY requirements.txt .
RUN pip install --upgrade pip && pip install -r requirements.txt

# Crear el workspace de ROS 2 y los paquetes necesarios
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    mkdir -p /ros2_ws/src && \
    cd /ros2_ws/src && \
    ros2 pkg create sample_pkg --build-type ament_python --dependencies rclpy std_msgs && \
    ros2 pkg create sample_interfaces --build-type ament_cmake && \
    mkdir -p /ros2_ws/src/sample_interfaces/srv && \
    mkdir -p /ros2_ws/src/sample_interfaces/msg"

# Instalar dependencias adicionales para ROS
RUN /opt/venv/bin/pip install catkin_pkg empy numpy lark
RUN pip uninstall -y em && pip install empy==3.3.4

# Inicializa y actualiza rosdep
RUN rosdep update && rosdep install --from-paths /ros2_ws --ignore-src -y

# Compilar el workspace de ROS 2
RUN bash -c "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && colcon build --symlink-install"

# Copiar el backend de FastAPI (se copia la carpeta completa 'app')
COPY app /app

WORKDIR /app

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Definir el comando de inicio para FastAPI, NoVNC, y x11vnc
ENTRYPOINT ["/entrypoint.sh"]
