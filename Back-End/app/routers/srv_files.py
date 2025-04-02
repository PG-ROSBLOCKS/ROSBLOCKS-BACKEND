# app/routers/srv_files.py
import os
from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
from ..config import settings

router = APIRouter()

def parse_srv_file(file_path: str):
    """
    Lee un archivo .srv y extrae las variables definidas en la sección de solicitud y respuesta.
    Se espera que el archivo esté dividido por '---' en dos secciones.
    Cada línea que no sea comentario ni vacía debe tener el formato: <tipo> <nombre>
    """
    try:
        with open(file_path, "r") as f:
            content = f.read()
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error al leer el archivo {file_path}: {e}")
    
    parts = content.split('---')
    if len(parts) != 2:
        raise HTTPException(status_code=400, detail=f"El archivo {file_path} no tiene un formato .srv válido")
    
    def parse_section(section: str):
        variables = []
        for line in section.splitlines():
            line = line.strip()
            # Ignorar líneas vacías o comentarios
            if not line or line.startswith("#"):
                continue
            tokens = line.split()
            if len(tokens) >= 2:
                var_type, var_name = tokens[0], tokens[1]
                variables.append({"type": var_type, "name": var_name})
        return variables
    
    return {
        "request": parse_section(parts[0]),
        "response": parse_section(parts[1])
    }

@router.get("/", response_model=dict)
async def check_srv_files():
    srv_dir = os.path.join(settings.INTERFACES_DIR, "srv")
    try:
        files = os.listdir(srv_dir)
        srv_files = [f for f in files if f.endswith(".srv")]
        files_info = []
        for f in srv_files:
            file_path = os.path.join(srv_dir, f)
            variables = parse_srv_file(file_path)
            files_info.append({
                "name": f,
                "variables": variables
            })
        return JSONResponse({"exists": bool(srv_files), "files": files_info})
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
