FROM ros:jazzy

# 1. Instalar dependencias de sistema necesarias
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    nodejs \
    npm

RUN apt update && apt install -y \
    python3-pip python3-dev python3-venv tmux \
    ros-jazzy-turtlesim ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

ENV PATH="/opt/venv/bin:$PATH"
ENV DISPLAY=novnc:0.0

# 2. Instalar configurable-http-proxy de forma global
RUN npm install -g configurable-http-proxy

# 3. Crear un entorno virtual para Python y actualizar pip
RUN python3 -m venv /opt/venv
RUN /opt/venv/bin/pip install --upgrade pip


RUN /opt/venv/bin/pip install jupyterhub
RUN /opt/venv/bin/pip install jupyter

RUN useradd testuser --create-home --shell /bin/bash
RUN echo 'testuser:passwd' | chpasswd


# 5. Ajustar PATH para que se incluyan /opt/venv/bin y /usr/local/bin (donde vive configurable-http-proxy)
ENV PATH="/opt/venv/bin:/usr/local/bin:$PATH"

COPY app ./app
COPY jupyterhub_config.py .
COPY requirements.txt /tmp/requirements.txt
RUN /opt/venv/bin/pip install -r /tmp/requirements.txt

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

# 6. Copiar un script de entrypoint
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# 7. Definir ENTRYPOINT y CMD por defecto
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["jupyterhub", "--ip", "0.0.0.0"]
