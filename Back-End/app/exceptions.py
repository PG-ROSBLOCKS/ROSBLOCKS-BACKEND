# app/exceptions.py
import logging
from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse

async def custom_exception_handler(request: Request, exc: Exception):
    logging.error(f"Error: {str(exc)}")
    if isinstance(exc, HTTPException):
        return JSONResponse(status_code=exc.status_code, content={"detail": exc.detail})
    return JSONResponse(status_code=500, content={"detail": "Internal Server Error"})
