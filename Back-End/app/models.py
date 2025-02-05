from pydantic import BaseModel

class CodeRequest(BaseModel):
    code: str  # Código Python enviado desde Angular

class CodeResponse(BaseModel):
    output: str
    error: str
