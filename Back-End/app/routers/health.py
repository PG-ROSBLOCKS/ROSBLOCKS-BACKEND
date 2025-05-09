# app/routers/health.py
from fastapi import APIRouter, HTTPException
router = APIRouter()

@router.get("/", response_model=dict)
async def health_check():
    return {"status": "ok"}