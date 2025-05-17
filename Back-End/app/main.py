from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from .config import settings
from .routers import upload, execution, export, srv_files, msg_files,reset, delete, ros_commands, health
from .exceptions import custom_exception_handler
import asyncio
import os
import httpx
import logging

app = FastAPI(title=settings.APP_NAME, version=settings.APP_VERSION)

# --- CORS ---
app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://rosblocks.com.co", "https://www.rosblocks.com.co"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- Routers ---
app.include_router(upload.router, prefix="/upload", tags=["Upload"])
app.include_router(execution.router, prefix="/execution", tags=["Execution"])
app.include_router(export.router, prefix="/export", tags=["Export"])
app.include_router(srv_files.router, prefix="/srvfiles", tags=["SrvFiles"])
app.include_router(msg_files.router, prefix="/msgfiles", tags=["MsgFiles"])
app.include_router(reset.router, prefix="/reset", tags=["Reset"])
app.include_router(delete.router, prefix="/delete", tags=["Delete"])
app.include_router(ros_commands.router, prefix="/ros_commands", tags=["RosCommands"])
app.include_router(health.router, prefix="/health", tags=["Health"])

# --- Global handler exception ---
app.add_exception_handler(Exception, custom_exception_handler)

# --- Inactivity ---
INACTIVITY_LIMIT = 1800  # seconds (5 minutes)
last_activity = asyncio.Event()

@app.middleware("http")
async def activity_middleware(request: Request, call_next):
    if not request.url.path.startswith("/health"):
        last_activity.set()
    response = await call_next(request)
    return response

async def get_task_arn_from_metadata() -> str:
    try:
        async with httpx.AsyncClient() as client:
            res = await client.get("http://169.254.170.2/v2/metadata", timeout=5)
            res.raise_for_status()
            metadata = res.json()
            return metadata.get("TaskARN", "UNKNOWN_TASK_ARN")
    except Exception as e:
        print("Error obtaining Task ARN:", e)
        return "UNKNOWN_TASK_ARN"

@app.on_event("startup")
async def start_inactivity_timer():
    async def monitor_inactivity():
        while True:
            last_activity.clear()
            try:
                await asyncio.wait_for(last_activity.wait(), timeout=INACTIVITY_LIMIT)
            except asyncio.TimeoutError:
                print("No activity detected in 5 minutes. Notifying shutdown...")

                try:
                    task_arn = await get_task_arn_from_metadata()
                    async with httpx.AsyncClient() as client:
                        response = await client.post(
                            f"{settings.SESSION_MANAGER_URL}/delete-task",
                            json={"task_arn": task_arn}
                        )
                        print("Task deletion notified:", response.status_code)
                except Exception as e:
                    print("Error notifying task deletion:", e)

                os._exit(0)

    asyncio.create_task(monitor_inactivity())
