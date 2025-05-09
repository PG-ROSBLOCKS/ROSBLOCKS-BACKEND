# app/config.py
from pydantic_settings import BaseSettings
from pydantic import ConfigDict
from typing import List


class Settings(BaseSettings):
    APP_NAME: str = "My FastAPI Project"
    APP_VERSION: str = "1.0.0"
    HOST: str = "0.0.0.0"
    PORT: int = 8000
    ALLOWED_ORIGINS: List[str] = ["http://localhost:4200"]

    # Rutas y otros parámetros
    SCRIPTS_DIR: str = "/ros2_ws/src/sample_pkg/sample_pkg"
    EXPORT_DIR: str = "/app/exported"
    INTERFACES_DIR: str = "/ros2_ws/src/sample_interfaces"
    SRV_DIR: str = "/ros2_ws/src/sample_interfaces/srv"
    MSG_DIR: str = "/ros2_ws/src/sample_interfaces/msg"
    LOG_DIR: str = "/ros2_ws/logs"
    CMAKE_FILE: str = "/ros2_ws/src/sample_interfaces/CMakeLists.txt"
    SESSION_MANAGER_URL: str= "https://api.rosblocks.com.co/api"

    model_config = ConfigDict(env_file=".env", extra='ignore')


settings = Settings()
