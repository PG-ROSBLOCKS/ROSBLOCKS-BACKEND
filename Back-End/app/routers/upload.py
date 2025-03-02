# app/routers/upload.py
from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
from models.requests import UploadRequest
from services.code_generator import generate_pub_sub, generate_service, generate_message
import logging

router = APIRouter()

@router.post("/", response_model=dict)
async def upload_code(request: UploadRequest):
    if request.type == "pub_sub":
        return await generate_pub_sub(request)
    elif request.type == "srv":
        return await generate_service(request)
    elif request.type == "msg":
        return await generate_message(request)
    else:
        raise HTTPException(status_code=400, detail="Tipo de request no soportado")
