# tests/test_routes.py
import pytest
from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)

def test_upload_invalid_type():
    response = client.post("/upload/", json={
        "file_name": "test.py",
        "code": "print('Hello')",
        "type": "invalid"
    })
    assert response.status_code == 400

# Se pueden agregar más pruebas según se requiera
