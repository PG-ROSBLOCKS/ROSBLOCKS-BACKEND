from fastapi import FastAPI, WebSocket, HTTPException
from fastapi.responses import JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from fastapi.websockets import WebSocketDisconnect
from pydantic import BaseModel
import asyncio
import os
import subprocess
import uuid
import json
from ros_files_modifiers import update_setup_py, update_package_xml
import logging
import traceback

app = FastAPI()

# Configuración de CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:4200"],  
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 📂 Directorio donde se almacenan los scripts dentro del paquete ROS 2
SCRIPTS_DIR = "/ros2_ws/src/sample_pkg/sample_pkg"

class UploadRequest(BaseModel):
    file_name: str
    code: str

import logging
import subprocess

# Configurar logging
logging.basicConfig(level=logging.INFO)

@app.post("/upload/")
async def upload_code(request: UploadRequest):
    file_path = os.path.join(SCRIPTS_DIR, request.file_name)
    node_name = request.file_name.replace(".py", "")

    try:
        logging.info(f"Recibiendo archivo: {file_path}")

        os.makedirs(SCRIPTS_DIR, exist_ok=True)

        with open(file_path, "w") as file:
            file.write(request.code)

        os.chmod(file_path, 0o755)  # Hacer el archivo ejecutable
        logging.info(f"Archivo guardado y hecho ejecutable: {file_path}")

        # Modificar setup.py
        setup_file = "/ros2_ws/src/sample_pkg/setup.py"
        update_setup_py(setup_file, node_name)
        logging.info("setup.py actualizado correctamente.")

        # Modificar package.xml
        package_xml_file = "/ros2_ws/src/sample_pkg/package.xml"
        update_package_xml(package_xml_file)
        logging.info("package.xml actualizado correctamente.")

        # Recompilar ROS 2 y capturar la salida
        result = subprocess.run(
            "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --symlink-install'",
            shell=True,
            check=False,
            capture_output=True,
            text=True
        )

        logging.info(f"Colcon build output:\n{result.stdout}")
        logging.error(f"Colcon build error:\n{result.stderr}")

        return JSONResponse({"message": "File uploaded and package rebuilt", "file": request.file_name})

    except Exception as e:
        logging.error(f"Unexpected error: {str(e)}")
        return JSONResponse(status_code=500, content={"error": "Unexpected error", "details": str(e)})



'''
@app.get("/execute/{file_name}")
async def execute_code(file_name: str):
    session_id = f"ros_session_{uuid.uuid4().hex[:8]}"

    try:
        subprocess.run("tmux start-server", shell=True, check=False)

        command = f"""
        tmux new-session -d -s {session_id} "bash -c '
        cd /ros2_ws/src/sample_pkg &&
        source /root/.bashrc &&
        source /opt/ros/jazzy/setup.bash &&
        source /ros2_ws/install/setup.bash &&
        export PYTHONUNBUFFERED=1 &&
        colcon build --symlink-install &&
        source /ros2_ws/install/setup.bash &&
        ros2 run sample_pkg {file_name[:-3]};
        exec bash'"
        """
        
        subprocess.run(command, shell=True, check=True)

        return JSONResponse({"message": "Execution started", "session_id": session_id})

    except subprocess.CalledProcessError as e:
        raise HTTPException(status_code=500, detail=f"Error executing script: {str(e)}")


@app.websocket("/ws/{session_id}")
async def websocket_endpoint(websocket: WebSocket, session_id: str):
    await websocket.accept()
    last_output_lines = []

    try:
        while True:
            # Capturar la salida de la sesión `tmux` en tiempo real
            command = f"tmux capture-pane -p -t {session_id}"
            process = await asyncio.create_subprocess_shell(
                command,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )

            stdout, stderr = await process.communicate()
            output = stdout.decode("utf-8").strip()

            if not output:  # Evita enviar respuestas vacías
                await asyncio.sleep(0.5)
                continue

            current_output_lines = output.split("\n")
            new_lines = current_output_lines[len(last_output_lines):]

            if new_lines:
                message = json.dumps({"output": "\n".join(new_lines)})
                await websocket.send_text(message)

            last_output_lines = current_output_lines
            await asyncio.sleep(0.5)

    except WebSocketDisconnect:
        print(f"Cliente desconectado de {session_id}")
'''

@app.get("/execute/{file_name}")
async def execute_code(file_name: str):
    session_id = f"ros_session_{uuid.uuid4().hex[:8]}"
    log_file = f"/ros2_ws/logs/{session_id}.log"

    try:
        subprocess.run("tmux start-server", shell=True, check=False)

        # Crear el directorio de logs si no existe
        os.makedirs("/ros2_ws/logs", exist_ok=True)

        # Asegurar que `tmux` usa `pipe-pane` para redirigir salida a un archivo
        command = f"""
        tmux new-session -d -s {session_id} "bash -c '
        source /ros2_ws/install/setup.bash &&
        export PYTHONUNBUFFERED=1 &&
        ros2 run sample_pkg {file_name[:-3]} 2>&1 | tee {log_file};
        exec bash'"
        """
        
        subprocess.run(command, shell=True, check=True)

        return JSONResponse({"message": "Execution started", "session_id": session_id})

    except subprocess.CalledProcessError as e:
        raise HTTPException(status_code=500, detail=f"Error executing script: {str(e)}")


@app.websocket("/ws/{session_id}")
async def websocket_endpoint(websocket: WebSocket, session_id: str):
    await websocket.accept()
    log_file = f"/ros2_ws/logs/{session_id}.log"
    
    try:
        last_pos = 0  # Posición inicial en el archivo

        while True:
            # Verificar si el archivo de logs existe
            if not os.path.exists(log_file):
                await asyncio.sleep(0.5)
                continue

            # Abrir el archivo y leer solo las nuevas líneas
            with open(log_file, "r") as f:
                f.seek(last_pos)  # Ir a la última posición
                new_output = f.read()
                last_pos = f.tell()  # Guardar la nueva posición

            if new_output.strip():  # Evitar enviar mensajes vacíos
                message = json.dumps({"output": new_output.strip()})
                await websocket.send_text(message)

            await asyncio.sleep(0.5)

    except WebSocketDisconnect:
        print(f"Cliente desconectado de {session_id}")
