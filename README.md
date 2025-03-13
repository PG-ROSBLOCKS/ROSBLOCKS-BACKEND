# ROSBlocks BACKEND

ROSBlocks BACKEND es una implementación modular de FastAPI que provee servicios para la generación y ejecución de código basado en bloques, integrándose con ROS2 para la generación de mensajes, servicios y nodos. Además, se integra con novnc para la simulación y muestra de la tortuga de Turtlesim en el front.

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

## Dependencias necesarias

- Se necesita tener instalado Docker en el sistema operativo

## Instalación de Docker
### Ubuntu / Debian
Sigue las instrucciones oficiales de Docker (https://docs.docker.com/desktop/setup/install/linux/ubuntu/) o ejecuta los siguientes comandos:

1. Agregar la clave GPG oficial de Docker:

```bash
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```
2. Agregar el repositorio de Docker a las fuentes de Apt:
```bash
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```
3. Instalar Docker Engine:
```bash
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```
4. Verificar la instalación:
```bash
sudo docker run hello-world
```
### Windows
- Descarga Docker Desktop para Windows desde el sitio oficial de Docker: https://docs.docker.com/desktop/setup/install/windows-install/. 
- Sigue el asistente de instalación. Asegúrate de habilitar WSL 2 durante la instalación.
- Una vez instalado, abre Docker Desktop y verifica que el servicio esté en ejecución.

## Descargar el repositorio

Una vez descargado el proyecto y descomprimido o en su defecto una vez se haya clonado:
```bash
git clone https://github.com/PG-ROSBLOCKS/ROSBLOCKS-BACKEND.git
```
## Ejecutar el proyecto
Desde la carpeta raíz (donde se encuentra el archivo docker-compose.yml), ejecuta:

```bash
docker-compose up --build
```
Este comando compilará las imágenes y levantará los servicios definidos.
- El backend de FastAPI se expondrá en http://localhost:8000.

## Endpoints y Documentación
Una vez que el contenedor esté corriendo, accede a la documentación interactiva de la API en:
- Swagger UI: http://localhost:8000/docs
