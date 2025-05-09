# app/services/execution_manager.py
import os, subprocess, uuid, asyncio, json, logging
from fastapi import HTTPException
from fastapi.responses import JSONResponse, FileResponse
from starlette.websockets import WebSocketDisconnect
from utils.ros_modifiers import update_cmake_lists_services, update_cmake_lists_messages
from config import settings

logging.basicConfig(level=logging.INFO)

async def execute_code(file_name: str):
    session_id = f"ros_session_{uuid.uuid4().hex[:8]}"
    log_file = os.path.join(settings.LOG_DIR, f"{session_id}.log")
    try:
        subprocess.run("tmux start-server", shell=True, check=False)
        os.makedirs(settings.LOG_DIR, exist_ok=True)
        command = f"""
        tmux new-session -d -s {session_id} "bash -c '
        source /ros2_ws/install/setup.bash &&
        export PYTHONUNBUFFERED=1 &&
        ros2 run sample_pkg {file_name[:-3]} 2>&1 | tee {log_file};'"
        """
        subprocess.run(command, shell=True, check=True)
        logging.info(f"TMUX INICIADO: Sesión {session_id} creada para {file_name}")
        return JSONResponse({"message": "Execution started", "session_id": session_id})
    except subprocess.CalledProcessError as e:
        raise HTTPException(status_code=500, detail=f"Error executing script: {str(e)}")

async def kill_execution(session_id: str):
    try:
        command = f"tmux kill-session -t {session_id}"
        subprocess.run(command, shell=True, check=True)
        return JSONResponse({"message": "Execution stopped", "session_id": session_id})
    except subprocess.CalledProcessError as e:
        raise HTTPException(status_code=500, detail=f"Error stopping execution: {str(e)}")

async def cleanup_workspace(file_name: str):
    node_name = file_name.replace(".py", "")
    file_path = os.path.join(settings.SCRIPTS_DIR, node_name + ".py")
    try:
        if os.path.exists(file_path):
            os.remove(file_path)
            logging.info(f"Archivo eliminado: {file_path}")
        else:
            logging.warning(f"Archivo no encontrado: {file_path}")
        setup_file = "/ros2_ws/src/sample_pkg/setup.py"

        from utils.ros_modifiers import update_setup_py, update_package_xml
        update_setup_py(setup_file, node_name, requestType="none", remove=True)

        package_xml_file = "/ros2_ws/src/sample_pkg/package.xml"
        update_package_xml(package_xml_file, remove=True)
        result = subprocess.run(
            "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --symlink-install'",
            shell=True, check=False, capture_output=True, text=True
        )
        logging.info(f"Colcon build output:\n{result.stdout}")
        logging.error(f"Colcon build error:\n{result.stderr}")
        return JSONResponse({"message": "Workspace cleaned successfully", "file": file_name})
    except Exception as e:
        logging.error(f"Error al limpiar el workspace: {str(e)}")
        return JSONResponse(status_code=500, content={"error": "Cleanup failed", "details": str(e)})

async def export_project(background_tasks):
    tar_path_local = "/tmp/ros2_ws_backend.tar.gz"
    workspace_path = "/ros2_ws/src/"
    try:
        subprocess.run(
            ["bash", "-c", f"tar -czvf {tar_path_local} {workspace_path}"],
            check=True
        )
    except subprocess.CalledProcessError as e:
        raise HTTPException(status_code=500, detail=f"Error al comprimir workspace: {e}")
    if not os.path.exists(tar_path_local):
        raise HTTPException(status_code=404, detail="No se encontró el archivo comprimido en backend")
    background_tasks.add_task(os.remove, tar_path_local)
    return FileResponse(
        path=tar_path_local,
        media_type="application/gzip",
        filename="ros2_ws.tar.gz"
    )

async def websocket_handler(websocket, session_id: str):
    await websocket.accept()
    log_file = os.path.join(settings.LOG_DIR, f"{session_id}.log")
    try:
        last_pos = 0
        while True:
            # Check if tmux session still exists
            check_command = f"tmux has-session -t {session_id}"
            process = await asyncio.create_subprocess_shell(
                check_command,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            await process.wait()

            if process.returncode != 0:
                logging.info(f"Tmux session {session_id} not found. Assuming terminated.")
                logging.info(f"TMUX FINALIZADO: Sesión {session_id} ya no existe.")
                await websocket.send_text(json.dumps({"status": "terminated", "session_id": session_id}))
                break  # Exit the loop as the session is gone

            # Read new log output
            if os.path.exists(log_file):
                with open(log_file, "r") as f:
                    f.seek(last_pos)
                    new_output = f.read()
                    if new_output: # Only update last_pos if something was read
                         last_pos = f.tell()

                if new_output.strip():
                    message = json.dumps({"output": new_output.strip()})
                    await websocket.send_text(message)

            await asyncio.sleep(0.5) # Check periodically

    except WebSocketDisconnect:
        logging.info(f"Cliente desconectado de {session_id}")
    except Exception as e:
        logging.error(f"WebSocket error for session {session_id}: {str(e)}")
        # Optionally notify the client about the error before closing
        try:
            await websocket.send_text(json.dumps({"status": "error", "detail": str(e)}))
        except Exception:
            pass # Ignore errors trying to send on a potentially closed socket
    finally:
        # Ensure the websocket is closed gracefully
        await websocket.close()
        logging.info(f"WebSocket connection closed for session {session_id}")

async def delete_srv_file(file_name: str):
    file_path = os.path.join(settings.SRV_DIR, file_name)
    logging.info(f"Eliminando srv: {file_path}")
    srv_name = file_name.replace(".srv", "")
    try:
        if os.path.exists(file_path):
            os.remove(file_path)
            logging.info(f"Archivo .srv eliminado: {file_path}")
        else:
            logging.warning(f"Archivo .srv no encontrado: {file_path}")

        cmake_file = settings.CMAKE_FILE
        update_cmake_lists_services(cmake_file, srv_name, remove=True)

        return JSONResponse({"message": ".srv file deleted successfully", "file": file_name})
    except Exception as e:
        logging.error(f"Error al eliminar el archivo .srv: {str(e)}")
        return JSONResponse(status_code=500, content={"error": "Deletion failed", "details": str(e)})

async def delete_msg_file(file_name: str):
    msg_name = file_name.replace(".msg", "")
    file_path = os.path.join(settings.MSG_DIR, file_name)
    try:
        if os.path.exists(file_path):
            os.remove(file_path)
            logging.info(f"Archivo .msg eliminado: {file_path}")
        else:
            logging.warning(f"Archivo .msg no encontrado: {file_path}")

        cmake_file = settings.CMAKE_FILE
        update_cmake_lists_messages(cmake_file, msg_name, remove=True)

        return JSONResponse({"message": ".msg file deleted successfully", "file": file_name})
    except Exception as e:
        logging.error(f"Error al eliminar el archivo .msg: {str(e)}")
        return JSONResponse(status_code=500, content={"error": "Deletion failed", "details": str(e)})
