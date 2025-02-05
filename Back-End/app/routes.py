from fastapi import APIRouter
from app.models import CodeRequest, CodeResponse
from app.run_code import run_python_code

router = APIRouter()

@router.post("/execute", response_model=CodeResponse)
def execute_code(request: CodeRequest):
    return run_python_code(request.code)
