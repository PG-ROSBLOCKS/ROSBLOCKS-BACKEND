# app/routers/ros_commands.py
from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
import logging
import subprocess

router = APIRouter()

@router.get("/view_topics", response_model=dict)
async def reset_container():
    result = subprocess.run(
        "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && ros2 topic list'",
        shell=True, check=False, capture_output=True, text=True
    )
    logging.info(f"ros2 topic list result:\n{result.stdout}")
    return {"message": "ROS2 TOPIC LIST EXECUTED", "output": result.stdout}

@router.get("stop_ros_daemon", response_model=dict)
async def stop_ros_daemon():
    result = subprocess.run(
        "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && ros2 daemon stop'",
        shell=True, check=False, capture_output=True, text=True
    )
    logging.info(f"ros2 daemon stop result:\n{result.stdout}")
    return {"message": "ROS2 DAEMON STOP EXECUTED", "output": result.stdout}
