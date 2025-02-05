from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from app.routes import router

app = FastAPI(title="Code Executor API")

# Configuración de CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:4200"],  # Asegúrate de que tu frontend Angular esté permitido
    allow_credentials=True,
    allow_methods=["*"],  # Permitir todos los métodos, incluidos OPTIONS
    allow_headers=["*"],  # Permitir todos los encabezados
)
# Registrar las rutas
app.include_router(router)

@app.get("/")
def read_root():
    return {"message": "API running"}
