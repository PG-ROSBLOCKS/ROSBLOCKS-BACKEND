# ROSBlocks Backend

**ROSBlocks Backend** is a modular FastAPI implementation that provides services for generating and executing ROS 2-based code from visual blocks. It integrates with the ROS 2 framework to support the creation of messages, services, and nodes. It also includes support for graphical simulation via noVNC and Turtlesim, enabling visual feedback in the frontend.

## Project Structure

- **app/**: Main application source code
  - **main.py**: Entry point
  - **config.py**: Global configuration
  - **models/**: Pydantic models
  - **routers/**: API endpoints grouped by functionality
  - **services/**: Business logic and backend operations
  - **utils/**: Utility functions and helpers
  - **exceptions.py**: Global exception handling
- **tests/**: Unit and integration tests
- **requirements.txt**: Project dependencies

## Requirements

- Docker must be installed on your system

## Docker Installation

### Ubuntu / Debian

Follow the [official Docker installation guide](https://docs.docker.com/desktop/setup/install/linux/ubuntu/) or run the commands below:

1. Add Docker's official GPG key:

```bash
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```

2. Add the Docker repository:

```bash
echo   "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu   $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" |   sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```

3. Install Docker Engine:

```bash
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

4. Verify the installation:

```bash
sudo docker run hello-world
```

### Windows

- Download Docker Desktop for Windows from the [official Docker site](https://docs.docker.com/desktop/setup/install/windows-install/)
- Follow the installer instructions, and enable WSL 2 when prompted
- Once installed, open Docker Desktop and confirm it is running

## Clone the Repository

```bash
git clone https://github.com/PG-ROSBLOCKS/ROSBLOCKS-BACKEND.git
cd ROSBLOCKS-BACKEND
```

## Run the Project

From the root directory (where `docker-compose.yml` is located), run:

```bash
docker-compose up --build
```

This will build the Docker images and start all defined services.
- FastAPI will be available at: `http://localhost:8000`

## API Documentation

Once the backend container is running, access the interactive API documentation at:
- Swagger UI: [http://localhost:8000/docs](http://localhost:8000/docs)
