# app/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from config import settings
from routers import upload, execution, export, srv_files, msg_files,reset, delete
from exceptions import custom_exception_handler
import logging

app = FastAPI(title=settings.APP_NAME, version=settings.APP_VERSION)

# Configuración de CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Incluir routers
app.include_router(upload.router, prefix="/upload", tags=["Upload"])
app.include_router(execution.router, prefix="/execution", tags=["Execution"])
app.include_router(export.router, prefix="/export", tags=["Export"])
app.include_router(srv_files.router, prefix="/srvfiles", tags=["SrvFiles"])
app.include_router(msg_files.router, prefix="/msgfiles", tags=["MsgFiles"])
app.include_router(reset.router, prefix="/reset", tags=["Reset"])
app.include_router(delete.router, prefix="/delete", tags=["Delete"])

# Manejador global de excepciones
app.add_exception_handler(Exception, custom_exception_handler)

if __name__ == '__main__':
    import uvicorn
    uvicorn.run(app, host=settings.HOST, port=settings.PORT)
