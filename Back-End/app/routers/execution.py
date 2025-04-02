# app/routers/execution.py
from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from ..services.execution_manager import execute_code, kill_execution, cleanup_workspace, websocket_handler
import logging

router = APIRouter()

@router.get("/execute/{file_name}", response_model=dict)
async def execute_endpoint(file_name: str):
    return await execute_code(file_name)

@router.websocket("/ws/{session_id}")
async def websocket_endpoint(websocket: WebSocket, session_id: str):
    await websocket_handler(websocket, session_id)

@router.get("/kill/{session_id}", response_model=dict)
async def kill_endpoint(session_id: str):
    return await kill_execution(session_id)

@router.delete("/cleanup/{file_name}", response_model=dict)
async def cleanup_endpoint(file_name: str):
    return await cleanup_workspace(file_name)
