# app/services/code_generator.py
import os, subprocess, logging
from fastapi.responses import JSONResponse
from models.requests import UploadRequest
from config import settings
from utils import ros_modifiers

logging.basicConfig(level=logging.INFO)

async def generate_service(request: UploadRequest):
    srv_file_path = os.path.join(settings.INTERFACES_DIR, "srv", request.file_name)
    service_name = request.file_name.replace(".srv", "")
    try:
        logging.info(f"Recibiendo archivo de servicio: {srv_file_path}")
        os.makedirs(os.path.join(settings.INTERFACES_DIR, "srv"), exist_ok=True)
        with open(srv_file_path, "w") as file:
            file.write(request.code)
        # Actualizar package.xml y CMakeLists.txt
        package_xml_file = os.path.join(settings.INTERFACES_DIR, "package.xml")
        ros_modifiers.update_package_xml_services(package_xml_file)
        logging.info("package.xml actualizado para servicios.")
        cmake_file = os.path.join(settings.INTERFACES_DIR, "CMakeLists.txt")
        ros_modifiers.update_cmake_lists_services(cmake_file, service_name)
        logging.info("CMakeLists.txt actualizado.")
        result = subprocess.run(
            "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --packages-select sample_interfaces'",
            shell=True, check=False, capture_output=True, text=True
        )
        logging.info(f"Colcon build output:\n{result.stdout}")
        logging.error(f"Colcon build error:\n{result.stderr}")
        return JSONResponse({"message": "Service uploaded and package rebuilt", "file": request.file_name})
    except Exception as e:
        logging.error(f"Unexpected error: {str(e)}")
        return JSONResponse(status_code=500, content={"error": "Unexpected error", "details": str(e)})

async def generate_message(request: UploadRequest):
    msg_file_path = os.path.join(settings.INTERFACES_DIR, "msg", request.file_name)
    message_name = request.file_name.replace(".msg", "")
    try:
        logging.info(f"Recibiendo archivo de mensaje: {msg_file_path}")
        os.makedirs(os.path.join(settings.INTERFACES_DIR, "msg"), exist_ok=True)
        with open(msg_file_path, "w") as file:
            file.write(request.code)
        package_xml_file = os.path.join(settings.INTERFACES_DIR, "package.xml")
        ros_modifiers.update_package_xml_services(package_xml_file)
        logging.info("package.xml actualizado para mensajes.")
        cmake_file = os.path.join(settings.INTERFACES_DIR, "CMakeLists.txt")
        ros_modifiers.update_cmake_lists_messages(cmake_file, message_name)
        logging.info("CMakeLists.txt actualizado.")
        result = subprocess.run(
            "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --packages-select sample_interfaces'",
            shell=True, check=False, capture_output=True, text=True
        )
        logging.info(f"Colcon build output:\n{result.stdout}")
        logging.error(f"Colcon build error:\n{result.stderr}")
        return JSONResponse({"message": "Msg uploaded and package rebuilt", "file": request.file_name})
    except Exception as e:
        logging.error(f"Unexpected error: {str(e)}")
        return JSONResponse(status_code=500, content={"error": "Unexpected error", "details": str(e)})

async def generate_pub_sub(request: UploadRequest):
    file_path = os.path.join(settings.SCRIPTS_DIR, request.file_name)
    node_name = request.file_name.replace(".py", "")
    try:
        logging.info(f"Recibiendo archivo: {file_path}")
        os.makedirs(settings.SCRIPTS_DIR, exist_ok=True)
        with open(file_path, "w") as file:
            file.write(request.code)
        # Crear __init__.py si no existe
        init_file = os.path.join(settings.SCRIPTS_DIR, "__init__.py")
        if not os.path.exists(init_file):
            with open(init_file, "w") as f:
                f.write("")
        os.chmod(file_path, 0o755)
        logging.info(f"Archivo guardado y hecho ejecutable: {file_path}")
        setup_file = "/ros2_ws/src/sample_pkg/setup.py"
        ros_modifiers.update_setup_py(setup_file, node_name, request.type)
        logging.info("setup.py actualizado.")
        package_xml_file = "/ros2_ws/src/sample_pkg/package.xml"
        ros_modifiers.update_package_xml(package_xml_file)
        logging.info("package.xml actualizado.")
        result = subprocess.run(
            "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --symlink-install'",
            shell=True, check=False, capture_output=True, text=True
        )
        logging.info(f"Colcon build output:\n{result.stdout}")
        logging.error(f"Colcon build error:\n{result.stderr}")
        return JSONResponse({"message": "File uploaded and package rebuilt", "file": request.file_name})
    except Exception as e:
        logging.error(f"Unexpected error: {str(e)}")
        return JSONResponse(status_code=500, content={"error": "Unexpected error", "details": str(e)})
    
async def generate_server(request: UploadRequest):
    file_path = os.path.join(settings.SCRIPTS_DIR, request.file_name)
    node_name = request.file_name.replace(".py", "")
    try:
        logging.info(f"Recibiendo archivo: {file_path}")
        os.makedirs(settings.SCRIPTS_DIR, exist_ok=True)
        with open(file_path, "w") as file:
            file.write(request.code)
        init_file = os.path.join(settings.SCRIPTS_DIR, "__init__.py")
        if not os.path.exists(init_file):
            with open(init_file, "w") as f:
                f.write("")
        os.chmod(file_path, 0o755)
        logging.info(f"Archivo guardado y hecho ejecutable: {file_path}")
        setup_file = "/ros2_ws/src/sample_pkg/setup.py"
        ros_modifiers.update_setup_py(setup_file, node_name, request.type)
        logging.info("setup.py actualizado.")

        package_xml_file = "/ros2_ws/src/sample_pkg/package.xml"
        ros_modifiers.update_package_xml(package_xml_file)
        logging.info("package.xml actualizado.")

        result = subprocess.run(
            "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --symlink-install'",
            shell=True, check=False, capture_output=True, text=True
        )
        logging.info(f"Colcon build output:\n{result.stdout}")
        logging.error(f"Colcon build error:\n{result.stderr}")
        return JSONResponse({"message": "File uploaded and package rebuilt", "file": request.file_name})
    except Exception as e:
        logging.error(f"Unexpected error: {str(e)}")
        return JSONResponse(status_code=500, content={"error": "Unexpected error", "details": str(e)})
    
async def generate_client(request: UploadRequest):
    file_path = os.path.join(settings.SCRIPTS_DIR, request.file_name)
    node_name = request.file_name.replace(".py", "")
    try:
        logging.info(f"Recibiendo archivo: {file_path}")
        os.makedirs(settings.SCRIPTS_DIR, exist_ok=True)
        with open(file_path, "w") as file:
            file.write(request.code)
        init_file = os.path.join(settings.SCRIPTS_DIR, "__init__.py")
        if not os.path.exists(init_file):
            with open(init_file, "w") as f:
                f.write("")
        os.chmod(file_path, 0o755)
        logging.info(f"Archivo guardado y hecho ejecutable: {file_path}")
        setup_file = "/ros2_ws/src/sample_pkg/setup.py"
    #     ros_modifiers.update_setup_py(setup_file, node_name, request.type)
    #     logging.info("setup.py actualizado.")
    #     package_xml_file = "/ros2_ws/src/sample_pkg/package.xml"
    #     ros_modifiers.update_package_xml(package_xml_file)
    #     logging.info("package.xml actualizado.")
    #     result = subprocess.run(
    #         "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --symlink-install'",
    #         shell=True, check=False, capture_output=True, text=True
    #     )
    #     logging.info(f"Colcon build output:\n{result.stdout}")
    #     logging.error(f"Colcon build error:\n{result.stderr}")
    #     return JSONResponse({"message": "File uploaded and package rebuilt", "file": request.file_name})
    except Exception as e:
        logging.error(f"Unexpected error: {str(e)}")
        return JSONResponse(status_code=500, content={"error": "Unexpected error", "details": str(e)})

