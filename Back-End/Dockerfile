FROM ros:jazzy

# Instalar dependencias necesarias
RUN apt update && apt install -y \
    python3-pip python3-dev python3-venv tmux \
    ros-jazzy-turtlesim ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

# Crear entorno virtual para FastAPI
RUN python3 -m venv /opt/venv && \
    /opt/venv/bin/pip install --upgrade pip


RUN git clone https://github.com/novnc/noVNC.git /opt/novnc && \
    cd /opt/novnc && git checkout v1.6.0



RUN sudo apt-get update \
    && sudo apt-get install xvfb -y \
    && sudo apt-get install x11vnc screen -y \
    && sudo apt-get install websockify -y \
    && sudo apt-get install fluxbox -y  \
    && sudo apt-get install qt5dxcb-plugin -y 

RUN apt-get update && apt-get install -y \
    libxcb-xinerama0 \
    libxcb-xinput0 \
    libxkbcommon-x11-0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-shape0 \
    libxcb-xfixes0 \
    libxcb-sync1 \
    libxcb-xkb1 \
    libxrender1 \
    libxi6 \
    libxtst6 \
    x11-utils 

ENV PATH="/opt/venv/bin:$PATH"

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

# Configurar el entorno de ROS 2 para el contenedor
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Copiar el backend de FastAPI (se copia la carpeta completa 'app')
COPY app /app

WORKDIR /app

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
# Establecer display virtual
ENV DISPLAY=:99
ENV XDG_RUNTIME_DIR=/tmp/runtime-root
RUN mkdir -p $XDG_RUNTIME_DIR
RUN chmod 700 "$XDG_RUNTIME_DIR"

# Definir el comando de inicio para FastAPI
# CMD ["/opt/venv/bin/uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
ENTRYPOINT [ "/entrypoint.sh" ]
