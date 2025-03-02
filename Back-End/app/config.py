# app/config.py
from pydantic import BaseSettings
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
    LOG_DIR: str = "/ros2_ws/logs"

    class Config:
        env_file = ".env"


settings = Settings()
