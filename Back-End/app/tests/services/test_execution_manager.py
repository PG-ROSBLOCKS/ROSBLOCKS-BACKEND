# Back-End/app/tests/services/test_execution_manager.py
import pytest
import subprocess
import os 
import json 
from unittest.mock import patch, MagicMock
from fastapi import HTTPException
from fastapi.responses import JSONResponse

from app.services import execution_manager

# Mock settings globally for this test module
@pytest.fixture(autouse=True)
def mock_settings(mocker):
    # Use os.path.join
    mocker.patch("app.services.execution_manager.settings.LOG_DIR", os.path.join("/fake","log","dir"))
    # Mock SCRIPTS_DIR or other if needed for other functions in this file

@pytest.fixture
def mock_os_makedirs(mocker):
    return mocker.patch("os.makedirs")

@pytest.fixture
def mock_subprocess_run(mocker):
    mock_result = MagicMock()
    # check=False in the call to tmux new-session, we do not need check_returncode
    # mock_result.check_returncode.return_value = None
    run_mock = mocker.patch("subprocess.run", return_value=mock_result)
    return run_mock

@pytest.fixture
def mock_uuid(mocker):
    mock_uuid_obj = MagicMock()
    mock_uuid_obj.hex = "12345678abcdef"
    return mocker.patch("uuid.uuid4", return_value=mock_uuid_obj)

@pytest.mark.asyncio
async def test_execute_code_success(mocker, mock_settings, mock_os_makedirs, mock_subprocess_run, mock_uuid):
    """Test the successful flow of execute_code."""
    file_name = "my_node.py"
    expected_session_id = "ros_session_12345678"
    # Use os.path.join for the log path
    log_dir = os.path.join("/fake","log","dir")
    expected_log_file = os.path.join(log_dir, f"{expected_session_id}.log")
    expected_node_name = "my_node"

    mocker.patch("app.services.execution_manager.logging") # Mock logging

    response = await execution_manager.execute_code(file_name)

    # Verifications
    mock_os_makedirs.assert_called_once_with(log_dir, exist_ok=True)
    
    # Verify subprocess.run calls
    assert mock_subprocess_run.call_count == 2
    # 1. tmux start-server
    mock_subprocess_run.assert_any_call("tmux start-server", shell=True, check=False)
    # 2. tmux new-session (ensure the log path in the command uses the correct separator)
    expected_tmux_command = f"""
        tmux new-session -d -s {expected_session_id} "bash -c '
        source /ros2_ws/install/setup.bash &&
        export PYTHONUNBUFFERED=1 &&
        ros2 run sample_pkg {expected_node_name} 2>&1 | tee {expected_log_file};'" 
        """
    # Compare command cleaning extra spaces
    actual_call_args = mock_subprocess_run.call_args_list[1].args[0]
    # Simple normalization for robust comparison
    normalized_expected = ' '.join(expected_tmux_command.split())
    normalized_actual = ' '.join(actual_call_args.split())
    assert normalized_actual == normalized_expected
    assert mock_subprocess_run.call_args_list[1].kwargs['shell'] is True
    # The check for new-session is True in the original code
    assert mock_subprocess_run.call_args_list[1].kwargs['check'] is True

    # Verify the response
    assert isinstance(response, JSONResponse)
    assert response.status_code == 200
    assert json.loads(response.body.decode()) == {"message":"Execution started","session_id":f"{expected_session_id}"}

@pytest.mark.asyncio
async def test_execute_code_subprocess_fails(mocker, mock_settings, mock_os_makedirs, mock_subprocess_run, mock_uuid):
    """Test execute_code when subprocess.run fails for tmux new-session."""
    file_name = "my_node.py"
    # Use os.path.join for the log path
    log_dir = os.path.join("/fake","log","dir")

    # Configure the mock to fail in the second call (tmux new-session)
    error_message = "tmux command failed"
    # The code uses check=True for new-session, so we simulate CalledProcessError
    mock_subprocess_run.side_effect = [
        MagicMock(), # Success for tmux start-server
        subprocess.CalledProcessError(1, "tmux new-session", stderr=error_message)
    ]

    mocker.patch("app.services.execution_manager.logging")

    with pytest.raises(HTTPException) as exc_info:
        await execution_manager.execute_code(file_name)

    assert exc_info.value.status_code == 500
    # The detail must contain the str of the original exception
    expected_detail_part = str(subprocess.CalledProcessError(1, 'tmux new-session', stderr=error_message))
    assert f"Error executing script: {expected_detail_part}" in exc_info.value.detail

    # Verify that makedirs and the first subprocess were attempted
    mock_os_makedirs.assert_called_once_with(log_dir, exist_ok=True)
    assert mock_subprocess_run.call_count == 2 # Both calls were attempted 