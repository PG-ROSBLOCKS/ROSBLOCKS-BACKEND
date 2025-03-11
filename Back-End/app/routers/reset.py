# app/routers/reset.py
from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
import docker
import logging
import subprocess

router = APIRouter()

@router.post("/", response_model=dict)
async def reset_container():
    result = subprocess.run(
            "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && ros2 service call /reset std_srvs/srv/Empty'",
            shell=True, check=False, capture_output=True, text=True
        )
    logging.info(f"Turtlesim reset result:\n{result.stdout}")
