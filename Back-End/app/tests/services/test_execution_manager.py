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

@pytest.mark.asyncio
async def test_kill_execution_success(mocker, mock_subprocess_run):
    """Test kill_execution successful flow."""
    session_id = "test_session_to_kill"
    # mock_subprocess_run is already a fixture, configure its return value if needed
    # or ensure it doesn't raise an exception for this test.
    # By default, the fixture mock_subprocess_run has a MagicMock that won't raise error for check=True
    # unless its check_returncode is configured to do so, or side_effect is used.

    mocker.patch("app.services.execution_manager.logging") # Mock logging

    response = await execution_manager.kill_execution(session_id)

    expected_command = f"tmux kill-session -t {session_id}"
    mock_subprocess_run.assert_called_once_with(expected_command, shell=True, check=True)
    
    assert isinstance(response, JSONResponse)
    assert response.status_code == 200
    content = json.loads(response.body.decode())
    assert content == {"message": "Execution stopped", "session_id": session_id}

@pytest.mark.asyncio
async def test_kill_execution_subprocess_fails(mocker, mock_subprocess_run):
    """Test kill_execution when subprocess.run fails."""
    session_id = "test_session_fail_kill"
    error_message = "tmux kill command failed"
    
    # Configure the mock_subprocess_run fixture to raise CalledProcessError
    mock_subprocess_run.side_effect = subprocess.CalledProcessError(1, f"tmux kill-session -t {session_id}", stderr=error_message)

    mocker.patch("app.services.execution_manager.logging") # Mock logging

    with pytest.raises(HTTPException) as exc_info:
        await execution_manager.kill_execution(session_id)

    expected_command = f"tmux kill-session -t {session_id}"
    mock_subprocess_run.assert_called_once_with(expected_command, shell=True, check=True)

    assert exc_info.value.status_code == 500
    # The detail should contain the string representation of the original CalledProcessError
    expected_detail_part = str(subprocess.CalledProcessError(1, f"tmux kill-session -t {session_id}", stderr=error_message))
    assert f"Error stopping execution: {expected_detail_part}" in exc_info.value.detail 

@pytest.mark.asyncio
async def test_cleanup_workspace_success_file_exists(mocker, mock_settings, mock_subprocess_run):
    """Test cleanup_workspace when file exists and all steps succeed."""
    file_name = "my_node.py"
    node_name = "my_node"
    scripts_dir = execution_manager.settings.SCRIPTS_DIR
    file_path = os.path.join(scripts_dir, file_name)

    mock_os_path_exists = mocker.patch("os.path.exists", return_value=True)
    mock_os_remove = mocker.patch("os.remove")
    mock_update_setup_py = mocker.patch("app.services.execution_manager.update_setup_py")
    mock_update_package_xml = mocker.patch("app.services.execution_manager.update_package_xml")
    # mock_subprocess_run is a fixture, configure for colcon build success
    # The fixture already returns a MagicMock that simulates success by default for check=False

    mocker.patch("app.services.execution_manager.logging")

    response = await execution_manager.cleanup_workspace(file_name)

    mock_os_path_exists.assert_called_once_with(file_path)
    mock_os_remove.assert_called_once_with(file_path)
    mock_update_setup_py.assert_called_once_with("/ros2_ws/src/sample_pkg/setup.py", node_name, requestType="none", remove=True)
    mock_update_package_xml.assert_called_once_with("/ros2_ws/src/sample_pkg/package.xml", remove=True)
    mock_subprocess_run.assert_called_once_with(
        "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --symlink-install'",
        shell=True, check=False, capture_output=True, text=True
    )

    assert isinstance(response, JSONResponse)
    assert response.status_code == 200
    content = json.loads(response.body.decode())
    assert content == {"message": "Workspace cleaned successfully", "file": file_name}

@pytest.mark.asyncio
async def test_cleanup_workspace_success_file_not_exists(mocker, mock_settings, mock_subprocess_run):
    """Test cleanup_workspace when file does not exist and other steps succeed."""
    file_name = "other_node.py"
    node_name = "other_node"
    scripts_dir = execution_manager.settings.SCRIPTS_DIR
    file_path = os.path.join(scripts_dir, file_name)

    mock_os_path_exists = mocker.patch("os.path.exists", return_value=False)
    mock_os_remove = mocker.patch("os.remove") # Should not be called
    mock_update_setup_py = mocker.patch("app.services.execution_manager.update_setup_py")
    mock_update_package_xml = mocker.patch("app.services.execution_manager.update_package_xml")

    mocker.patch("app.services.execution_manager.logging")

    response = await execution_manager.cleanup_workspace(file_name)

    mock_os_path_exists.assert_called_once_with(file_path)
    mock_os_remove.assert_not_called()
    mock_update_setup_py.assert_called_once_with("/ros2_ws/src/sample_pkg/setup.py", node_name, requestType="none", remove=True)
    mock_update_package_xml.assert_called_once_with("/ros2_ws/src/sample_pkg/package.xml", remove=True)
    mock_subprocess_run.assert_called_once_with(
        "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --symlink-install'",
        shell=True, check=False, capture_output=True, text=True
    )

    assert isinstance(response, JSONResponse)
    assert response.status_code == 200
    content = json.loads(response.body.decode())
    assert content == {"message": "Workspace cleaned successfully", "file": file_name}

@pytest.mark.asyncio
async def test_cleanup_workspace_update_setup_py_fails(mocker, mock_settings):
    """Test cleanup_workspace when update_setup_py raises an exception."""
    file_name = "fail_setup.py"
    node_name = "fail_setup"
    scripts_dir = execution_manager.settings.SCRIPTS_DIR
    file_path = os.path.join(scripts_dir, file_name)

    mocker.patch("os.path.exists", return_value=True) # Assume file exists for this test path
    mocker.patch("os.remove")
    mock_update_setup_py = mocker.patch("app.services.execution_manager.update_setup_py", side_effect=Exception("Setup.py update error"))
    # update_package_xml and subprocess.run should not be called if update_setup_py fails early
    mock_update_package_xml = mocker.patch("app.services.execution_manager.update_package_xml") 
    mock_colcon_build = mocker.patch("subprocess.run")

    mock_log_error = mocker.patch("app.services.execution_manager.logging.error")

    response = await execution_manager.cleanup_workspace(file_name)

    mock_update_setup_py.assert_called_once_with("/ros2_ws/src/sample_pkg/setup.py", node_name, requestType="none", remove=True)
    mock_update_package_xml.assert_not_called() # Should not be reached
    mock_colcon_build.assert_not_called() # Should not be reached
    mock_log_error.assert_called_once_with(f"Error al limpiar el workspace: Setup.py update error")

    assert isinstance(response, JSONResponse)
    assert response.status_code == 500
    content = json.loads(response.body.decode())
    assert content == {"error": "Cleanup failed", "details": "Setup.py update error"}

@pytest.mark.asyncio
async def test_cleanup_workspace_update_package_xml_fails(mocker, mock_settings):
    """Test cleanup_workspace when update_package_xml raises an exception."""
    file_name = "fail_package_xml.py"
    node_name = "fail_package_xml"

    mocker.patch("os.path.exists", return_value=True)
    mocker.patch("os.remove")
    mocker.patch("app.services.execution_manager.update_setup_py") # Assumed to succeed
    mock_update_package_xml = mocker.patch("app.services.execution_manager.update_package_xml", side_effect=Exception("Package.xml update error"))
    mock_colcon_build = mocker.patch("subprocess.run")

    mock_log_error = mocker.patch("app.services.execution_manager.logging.error")

    response = await execution_manager.cleanup_workspace(file_name)

    mock_update_package_xml.assert_called_once_with("/ros2_ws/src/sample_pkg/package.xml", remove=True)
    mock_colcon_build.assert_not_called() # Should not be reached
    mock_log_error.assert_called_once_with(f"Error al limpiar el workspace: Package.xml update error")

    assert isinstance(response, JSONResponse)
    assert response.status_code == 500
    content = json.loads(response.body.decode())
    assert content == {"error": "Cleanup failed", "details": "Package.xml update error"}

@pytest.mark.asyncio
async def test_cleanup_workspace_colcon_build_fails(mocker, mock_settings, mock_subprocess_run):
    # Note: mock_subprocess_run is the fixture for subprocess.run
    """Test cleanup_workspace when colcon build fails (returns stderr)."""
    file_name = "colcon_fail_node.py"
    node_name = "colcon_fail_node"

    mocker.patch("os.path.exists", return_value=True)
    mocker.patch("os.remove")
    mocker.patch("app.services.execution_manager.update_setup_py")
    mocker.patch("app.services.execution_manager.update_package_xml")
    
    # Configure the existing mock_subprocess_run fixture for this specific test
    mock_colcon_result = MagicMock()
    mock_colcon_result.stdout = "Build partially successful"
    mock_colcon_result.stderr = "Error during colcon build!"
    mock_subprocess_run.return_value = mock_colcon_result # This mock is for the colcon build call

    mock_log_error = mocker.patch("app.services.execution_manager.logging.error")
    mocker.patch("app.services.execution_manager.logging.info") # Mock info as well

    response = await execution_manager.cleanup_workspace(file_name)

    mock_subprocess_run.assert_called_once_with(
        "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --symlink-install'",
        shell=True, check=False, capture_output=True, text=True
    )
    # Check that the specific colcon error was logged
    mock_log_error.assert_any_call(f"Colcon build error:\n{mock_colcon_result.stderr}") 

    assert isinstance(response, JSONResponse)
    # The original function returns 200 even if colcon build logs an error, due to check=False and no explicit error raising for this.
    assert response.status_code == 200 
    content = json.loads(response.body.decode())
    assert content == {"message": "Workspace cleaned successfully", "file": file_name} 