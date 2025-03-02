# app/models/requests.py
from pydantic import BaseModel

class UploadRequest(BaseModel):
    file_name: str
    code: str
    type: str  # Ej.: "pub_sub", "srv", "msg"
