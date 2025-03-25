# app/routers/srv_files.py
import os
from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
from config import settings

router = APIRouter()

def parse_msg_file(file_path: str):
    """
    Lee un archivo .msg y extrae las variables definidas.
    Cada línea que no sea comentario ni vacía debe tener el formato: <tipo> <nombre>
    """
    try:
        with open(file_path, "r") as f:
            content = f.read()
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error al leer el archivo {file_path}: {e}")

    variables = []
    for line in content.splitlines():
        line = line.strip()
        # Ignorar líneas vacías o comentarios
        if not line or line.startswith("#"):
            continue
        tokens = line.split()
        if len(tokens) >= 2:
            var_type, var_name = tokens[0], tokens[1]
            variables.append({"type": var_type, "name": var_name})
    
    return {
        "fields": variables
    }


@router.get("/", response_model=dict)
async def check_msg_files():
    msg_dir = os.path.join(settings.INTERFACES_DIR, "msg")
    try:
        files = os.listdir(msg_dir)
        msg_files = [f for f in files if f.endswith(".msg")]
        files_info = []
        for f in msg_files:
            file_path = os.path.join(msg_dir, f)
            variables = parse_msg_file(file_path)
            files_info.append({
                "name": os.path.splitext(f)[0], 
                "fields": variables["fields"]
            })
        return JSONResponse({"exists": bool(msg_files), "files": files_info})
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


