# app/routers/reset.py
from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
import docker
import logging

router = APIRouter()

@router.post("/", response_model=dict)
async def reset_container():
    """
    Reinicia el contenedor 'novnc' utilizando la API de Docker.
    """
    try:
        client = docker.from_env()
        container = client.containers.get("back-end-novnc-1")
        container.restart()
        return {"status": "Contenedor 'novnc' reiniciado exitosamente"}
    except Exception as error:
        logging.error(f"Error al reiniciar el contenedor 'novnc': {error}")
        raise HTTPException(status_code=500, detail=f"Error al reiniciar el contenedor: {error}")
