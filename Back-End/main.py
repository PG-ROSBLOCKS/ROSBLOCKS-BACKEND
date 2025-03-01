from fastapi import FastAPI, WebSocket, HTTPException, BackgroundTasks
from fastapi.responses import JSONResponse, FileResponse
from fastapi.middleware.cors import CORSMiddleware
from fastapi.websockets import WebSocketDisconnect
from pydantic import BaseModel
import asyncio
import os
import subprocess
import uuid
import json
from ros_files_modifiers import *
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

# Directorio donde se almacenan los scripts dentro del paquete ROS 2
SCRIPTS_DIR = "/ros2_ws/src/sample_pkg/sample_pkg"
EXPORT_DIR = "/app/exported"
INTERFACES_DIR = "/ros2_ws/src/sample_interfaces"

class UploadRequest(BaseModel):
    file_name: str
    code: str
    type: str

import logging
import subprocess

# Configurar logging
logging.basicConfig(level=logging.INFO)

@app.post("/upload/")
async def upload_code(request: UploadRequest):
    if(request.type == "pub_sub"):
        return await generate_pub_sub(request)
    elif(request.type == "srv"):
        return await generate_service(request)
    elif(request.type == "msg"):
        return await generate_message(request)
        
async def generate_service(request: UploadRequest):
    srv_file_path = os.path.join(INTERFACES_DIR, "srv", request.file_name)
    service_name = request.file_name.replace(".srv", "")

    try:
        logging.info(f"Recibiendo archivo de servicio: {srv_file_path}")

        os.makedirs(os.path.join(INTERFACES_DIR, "srv"), exist_ok=True)

        with open(srv_file_path, "w") as file:
            file.write(request.code)

        # Modificar package.xml
        package_xml_file = "/ros2_ws/src/sample_interfaces/package.xml"
        update_package_xml_services(package_xml_file)
        logging.info("package.xml actualizado correctamente para servicios.")

        # Modificar CMakeLists.txt
        cmake_file = "/ros2_ws/src/sample_interfaces/CMakeLists.txt"
        update_cmake_lists_services(cmake_file, service_name)
        logging.info("CMakeLists.txt actualizado correctamente.")

        # Recompilar solo el paquete de interfaces
        result = subprocess.run(
            "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --packages-select sample_interfaces'",
            shell=True,
            check=False,
            capture_output=True,
            text=True
        )

        logging.info(f"Colcon build output:\n{result.stdout}")
        logging.error(f"Colcon build error:\n{result.stderr}")

        return JSONResponse({"message": "Service uploaded and package rebuilt", "file": request.file_name})

    except Exception as e:
        logging.error(f"Unexpected error: {str(e)}")
        return JSONResponse(status_code=500, content={"error": "Unexpected error", "details": str(e)})
    
async def generate_message(request: UploadRequest):
    msg_file_path = os.path.join(INTERFACES_DIR, "msg", request.file_name)
    service_name = request.file_name.replace(".msg", "")

    try:
        logging.info(f"Recibiendo archivo de mensaje: {msg_file_path}")

        os.makedirs(os.path.join(INTERFACES_DIR, "msg"), exist_ok=True)

        with open(msg_file_path, "w") as file:
            file.write(request.code)

        # Modificar package.xml
        package_xml_file = "/ros2_ws/src/sample_interfaces/package.xml"
        update_package_xml_services(package_xml_file) # Se usa la misma función que para servicios
        logging.info("package.xml actualizado correctamente para mensajes.")

        # Modificar CMakeLists.txt
        cmake_file = "/ros2_ws/src/sample_interfaces/CMakeLists.txt"
        update_cmake_lists_messages(cmake_file, service_name)
        logging.info("CMakeLists.txt actualizado correctamente.")

        # Recompilar solo el paquete de interfaces
        result = subprocess.run(
            "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --packages-select sample_interfaces'",
            shell=True,
            check=False,
            capture_output=True,
            text=True
        )

        logging.info(f"Colcon build output:\n{result.stdout}")
        logging.error(f"Colcon build error:\n{result.stderr}")

        return JSONResponse({"message": "Msg uploaded and package rebuilt", "file": request.file_name})

    except Exception as e:
        logging.error(f"Unexpected error: {str(e)}")
        return JSONResponse(status_code=500, content={"error": "Unexpected error", "details": str(e)})
    
async def generate_pub_sub(request: UploadRequest):
    file_path = os.path.join(SCRIPTS_DIR, request.file_name)
    node_name = request.file_name.replace(".py", "")

    try:
        logging.info(f"Recibiendo archivo: {file_path}")

        os.makedirs(SCRIPTS_DIR, exist_ok=True)

        with open(file_path, "w") as file:
            file.write(request.code)
        
        #TODO: Funciona pero está creando algo que se soobreescribió, toca ver la forma de no sobreescribir usando el volumen
        init_file = os.path.join(SCRIPTS_DIR, "__init__.py")
        # Verificar y crear __init__.py si no existe
        if not os.path.exists(init_file):
            os.makedirs(SCRIPTS_DIR, exist_ok=True)
            with open(init_file, "w") as f:
                f.write("")  # Crea un archivo vacío

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
        ros2 run sample_pkg {file_name[:-3]} 2>&1 | tee {log_file};'"
        """ #Se agrega exec bash para congelar tmux para debbugear
        
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

@app.get("/kill/{session_id}")
async def kill_execution(session_id: str):
    try:
        print(f"Recibiendo solicitud para matar sesión: {session_id}")  # <-- DEBUG
        command = f"tmux kill-session -t {session_id}"
        subprocess.run(command, shell=True, check=True)

        return JSONResponse({"message": "Execution stopped", "session_id": session_id})

    except subprocess.CalledProcessError as e:
        print(f"Error al matar la sesión: {str(e)}")  # <-- DEBUG
        raise HTTPException(status_code=500, detail=f"Error stopping execution: {str(e)}")

@app.delete("/cleanup/{file_name}")
async def cleanup_workspace(file_name: str):
    node_name = file_name.replace(".py", "")
    file_path = os.path.join(SCRIPTS_DIR, node_name + ".py")

    try:
        # Eliminar el archivo Python
        if os.path.exists(file_path):
            os.remove(file_path)
            logging.info(f"Archivo eliminado: {file_path}")
        else:
            logging.warning(f"Archivo no encontrado: {file_path}")

        # Limpiar `setup.py`
        setup_file = "/ros2_ws/src/sample_pkg/setup.py"
        update_setup_py(setup_file, node_name, remove=True) 

        # Limpiar `package.xml`
        package_xml_file = "/ros2_ws/src/sample_pkg/package.xml"
        update_package_xml(package_xml_file, remove=True) 

        # Recompilar el workspace ROS 2 después de la limpieza
        result = subprocess.run(
            "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --symlink-install'",
            shell=True,
            check=False,
            capture_output=True,
            text=True
        )

        logging.info(f"Colcon build output:\n{result.stdout}")
        logging.error(f"Colcon build error:\n{result.stderr}")

        return JSONResponse({"message": "Workspace cleaned successfully", "file": file_name})

    except Exception as e:
        logging.error(f"Error al limpiar el workspace: {str(e)}")
        return JSONResponse(status_code=500, content={"error": "Cleanup failed", "details": str(e)})


@app.get("/export-project/")
async def export_project(background_tasks: BackgroundTasks):

    tar_path_local = "/tmp/ros2_ws_backend.tar.gz"
    workspace_path = "/ros2_ws/src/sample_pkg/"

    # 1) Comprimir /ros2_ws dentro del mismo contenedor "backend"
    try:
        subprocess.run(
            ["bash", "-c", f"tar -czvf {tar_path_local} {workspace_path}"],
            check=True
        )
    except subprocess.CalledProcessError as e:
        raise HTTPException(status_code=500, detail=f"Error al comprimir workspace: {e}")

    # 2) Verificar que el tar se generó
    if not os.path.exists(tar_path_local):
        raise HTTPException(status_code=404, detail="No se encontró el archivo comprimido en backend")

    # 3) Retornar el .tar.gz como un archivo descargable
    background_tasks.add_task(os.remove, tar_path_local)
    return FileResponse(
        path=tar_path_local,
        media_type="application/gzip",
        filename="ros2_ws.tar.gz"
    )
