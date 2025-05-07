# tests/test_routes.py
from unittest.mock import patch, AsyncMock
import pytest
from fastapi import HTTPException 
from fastapi.responses import JSONResponse

from app.models.requests import UploadRequest

# --- Test for /upload endpoint --- 

def test_upload_invalid_type(client):
    response = client.post("/upload/", json={
        "file_name": "test.py",
        "code": "print('Hello')",
        "type": "invalid_type_should_fail"
    })
    assert response.status_code == 400
    assert response.json() == {"detail": "Unsupported request type"}

@pytest.mark.parametrize("upload_type, service_function_name", [
    ("pub_sub", "generate_pub_sub"),
    ("srv", "generate_service"),
    ("msg", "generate_message"),
    ("server_node", "generate_server"), # Assuming it starts with 'server'
    ("client_node", "generate_client"), # Assuming it starts with 'client'
])
@patch("app.routers.upload.logging") # Mock logging in the router
def test_upload_valid_types_success(mock_logging, upload_type, service_function_name, client, mocker):
    """Test success for multiple valid types in /upload, mocking the service."""
    # Mock the corresponding service function
    mock_service_func = AsyncMock(return_value=JSONResponse(
        status_code=200, 
        content={"message": "Mock Success", "file": "test_file.xyz"}
    ))
    mocker.patch(f"app.routers.upload.{service_function_name}", mock_service_func)

    payload = {
        "file_name": "test_file.xyz",
        "code": "sample code",
        "type": upload_type
    }
    response = client.post("/upload/", json=payload)

    # Verify that the service was called correctly
    mock_service_func.assert_called_once()
    # The argument must be an UploadRequest object
    call_args, _ = mock_service_func.call_args
    assert isinstance(call_args[0], UploadRequest)
    assert call_args[0].file_name == payload["file_name"]
    assert call_args[0].code == payload["code"]
    assert call_args[0].type == payload["type"]

    # Verify the API response
    assert response.status_code == 200
    assert response.json() == {"message": "Mock Success", "file": "test_file.xyz"}

@patch("app.routers.upload.generate_pub_sub", new_callable=AsyncMock) # Mock a specific function
@patch("app.routers.upload.logging")
def test_upload_service_exception(mock_logging, mock_generate_pub_sub, client, mocker):
    """Test /upload when the underlying service raises an exception."""
    # Configure the mock to raise a simulated exception
    error_detail = "Service layer error occurred"
    # It could be an HTTPException or a generic exception
    mock_generate_pub_sub.side_effect = HTTPException(status_code=500, detail=error_detail)
    # Or: mock_generate_pub_sub.side_effect = Exception("Generic error")

    payload = {
        "file_name": "fail_node.py",
        "code": "some code",
        "type": "pub_sub" # Type must match the mocked function
    }
    response = client.post("/upload/", json=payload)

    # Verify the service was called
    mock_generate_pub_sub.assert_called_once()

    # Verify the error response (handled by the global exception handler or FastAPI)
    assert response.status_code == 500
    # Verify that the response matches the FastAPI format for HTTPException
    assert response.json() == {"detail": error_detail}

# --- Tests for /execution endpoints --- 

@patch("app.routers.execution.execute_code", new_callable=AsyncMock) 
@patch("app.routers.execution.logging") # Mock logging if necessary
def test_execute_endpoint_success(mock_logging, mock_execute_code, client):
    """Test GET /execution/execute/{file_name} with success."""
    file_name = "some_node.py"
    session_id = "fake_session_123"
    mock_execute_code.return_value = JSONResponse(
        status_code=200,
        content={"message": "Execution started", "session_id": session_id}
    )

    response = client.get(f"/execution/execute/{file_name}")

    mock_execute_code.assert_called_once_with(file_name)
    assert response.status_code == 200
    assert response.json() == {"message": "Execution started", "session_id": session_id}

@patch("app.routers.execution.execute_code", new_callable=AsyncMock)
@patch("app.routers.execution.logging")
def test_execute_endpoint_service_fails(mock_logging, mock_execute_code, client):
    """Test GET /execution/execute/{file_name} when the service fails."""
    file_name = "fail_node.py"
    error_detail = "Failed to start tmux session"
    mock_execute_code.side_effect = HTTPException(status_code=500, detail=error_detail)

    response = client.get(f"/execution/execute/{file_name}")

    mock_execute_code.assert_called_once_with(file_name)
    assert response.status_code == 500
    # Verify detail format
    assert response.json() == {"detail": error_detail}


@patch("app.routers.execution.kill_execution", new_callable=AsyncMock)
@patch("app.routers.execution.logging")
def test_kill_endpoint_success(mock_logging, mock_kill_execution, client):
    """Test GET /execution/kill/{session_id} with success."""
    session_id = "fake_session_to_kill"
    mock_kill_execution.return_value = JSONResponse(
        status_code=200,
        content={"message": "Execution stopped", "session_id": session_id}
    )

    response = client.get(f"/execution/kill/{session_id}")

    mock_kill_execution.assert_called_once_with(session_id)
    assert response.status_code == 200
    assert response.json() == {"message": "Execution stopped", "session_id": session_id}

@patch("app.routers.execution.kill_execution", new_callable=AsyncMock)
@patch("app.routers.execution.logging")
def test_kill_endpoint_service_fails(mock_logging, mock_kill_execution, client):
    """Test GET /execution/kill/{session_id} when the service fails."""
    session_id = "nonexistent_session"
    error_detail = "Session not found or failed to kill"
    mock_kill_execution.side_effect = HTTPException(status_code=500, detail=error_detail)

    response = client.get(f"/execution/kill/{session_id}")

    mock_kill_execution.assert_called_once_with(session_id)
    assert response.status_code == 500
    # Verify detail format
    assert response.json() == {"detail": error_detail}

@patch("app.routers.execution.cleanup_workspace", new_callable=AsyncMock)
@patch("app.routers.execution.logging")
def test_cleanup_endpoint_success(mock_logging, mock_cleanup_workspace, client):
    """Test DELETE /execution/cleanup/{file_name} with success."""
    file_name = "node_to_clean.py"
    mock_cleanup_workspace.return_value = JSONResponse(
        status_code=200,
        content={"message": "Workspace cleaned successfully", "file": file_name}
    )

    response = client.delete(f"/execution/cleanup/{file_name}")

    mock_cleanup_workspace.assert_called_once_with(file_name)
    assert response.status_code == 200
    assert response.json() == {"message": "Workspace cleaned successfully", "file": file_name}

@patch("app.routers.execution.cleanup_workspace", new_callable=AsyncMock)
@patch("app.routers.execution.logging")
def test_cleanup_endpoint_service_fails(mock_logging, mock_cleanup_workspace, client):
    """Test DELETE /execution/cleanup/{file_name} when the service fails."""
    file_name = "node_with_issues.py"
    error_detail = "Failed to remove files or rebuild"
    mock_cleanup_workspace.side_effect = HTTPException(status_code=500, detail=error_detail)

    response = client.delete(f"/execution/cleanup/{file_name}")

    mock_cleanup_workspace.assert_called_once_with(file_name)
    assert response.status_code == 500
    # Verify detail format
    assert response.json() == {"detail": error_detail}

