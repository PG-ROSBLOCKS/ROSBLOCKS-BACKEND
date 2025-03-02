# ROSBlocks BACKEND

Este proyecto es una implementación modular de FastAPI con servicios para la generación y ejecución de código basado en bloques.

## Estructura del Proyecto

- **app/**: Contiene el código fuente de la aplicación.
  - **main.py**: Punto de entrada.
  - **config.py**: Configuraciones globales.
  - **models/**: Modelos Pydantic.
  - **routers/**: Endpoints organizados por funcionalidad.
  - **services/**: Lógica de negocio.
  - **utils/**: Funciones auxiliares.
  - **exceptions.py**: Manejo de errores global.
- **tests/**: Pruebas unitarias e integradas.
- **requirements.txt**: Dependencias del proyecto.

## Cómo ejecutar

Instala las dependencias y ejecuta la aplicación:

```bash
pip install -r requirements.txt
uvicorn app.main:app --reload
