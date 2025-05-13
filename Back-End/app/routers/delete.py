from ..services.execution_manager import delete_srv_file, delete_msg_file
from fastapi import APIRouter, HTTPException
import logging

router = APIRouter()

@router.delete("/interfaces/{fileType}/{fileName}")
async def delete_interface(fileType: str, fileName: str):
    logging.info("Solicitud de eliminación recibida: fileType=%s, fileName=%s", fileType, fileName)
    if fileType.lower() not in ("srv", "msg"):
        logging.error("Tipo de archivo inválido recibido: %s", fileType)
        raise HTTPException(status_code=400, detail="Tipo de archivo inválido. Use 'srv' o 'msg'.")
    fileName = fileName[0].upper() + fileName[1:] if fileName else fileName
    extension = f".{fileType.lower()}"
    if not fileName.endswith(extension):
        fileName += extension
    if fileType.lower() == "srv":
        return await delete_srv_file(fileName)
    elif fileType.lower() == "msg":
        return await delete_msg_file(fileName)
