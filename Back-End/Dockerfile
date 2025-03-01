FROM ros:jazzy

# Instalar dependencias necesarias
RUN apt update && apt install -y \
    python3-pip python3-dev python3-venv tmux \
    ros-jazzy-turtlesim ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*


# Crear entorno virtual para FastAPI
RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Instalar FastAPI y Uvicorn
COPY requirements.txt .
RUN pip install --upgrade pip && pip install -r requirements.txt

# Crear el paquete ROS 2 en la imagen
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    mkdir -p /ros2_ws/src && \
    cd /ros2_ws/src && \
    ros2 pkg create sample_pkg --build-type ament_python --dependencies rclpy std_msgs && \
    ros2 pkg create sample_interfaces --build-type ament_cmake && \
    mkdir -p /ros2_ws/src/sample_interfaces/srv && \
    mkdir -p /ros2_ws/src/sample_interfaces/msg"

RUN /opt/venv/bin/pip install catkin_pkg
RUN /opt/venv/bin/pip install empy
RUN pip uninstall -y em && pip install empy==3.3.4
RUN pip install numpy
RUN pip install lark
# Inicializa y actualiza rosdep
RUN rosdep update && rosdep install --from-paths /ros2_ws --ignore-src -y

# Compilar el paquete de ROS 2 asegurando que se use BASH
RUN bash -c "source /opt/ros/jazzy/setup.bash && cd /ros2_ws && colcon build --symlink-install"

# Configurar el entorno de ROS 2 para el contenedor
RUN echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Copiar el backend de FastAPI
COPY main.py /app/main.py
COPY ros_files_modifiers.py /app/ros_files_modifiers.py 
WORKDIR /app

# Definir el comando de inicio para FastAPI
CMD ["/opt/venv/bin/uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]