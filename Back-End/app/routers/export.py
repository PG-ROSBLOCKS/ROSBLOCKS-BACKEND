# app/routers/export.py
from fastapi import APIRouter, BackgroundTasks, HTTPException
from fastapi.responses import FileResponse
from ..services.execution_manager import export_project
import logging

router = APIRouter()

@router.get("/", response_model=dict)
async def export_project_endpoint(background_tasks: BackgroundTasks):
    return await export_project(background_tasks)
