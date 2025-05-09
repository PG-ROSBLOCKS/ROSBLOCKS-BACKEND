# Back-End/app/tests/services/test_code_generator.py
import pytest
import os
import json
from unittest.mock import patch, mock_open, AsyncMock, MagicMock
from fastapi.responses import JSONResponse

# Assuming your UploadRequest is in models.requests 
from app.models.requests import UploadRequest 
from app.services import code_generator

# Mock settings before importing the service that uses it
@pytest.fixture(autouse=True)
def mock_settings(mocker):
    mocker.patch("app.services.code_generator.settings.SCRIPTS_DIR", os.path.join("/fake", "scripts", "dir"))
    mocker.patch("app.services.code_generator.settings.INTERFACES_DIR", os.path.join("/fake", "interfaces", "dir"))
    # Mock other configurations if needed for other tests

@pytest.fixture
def mock_ros_modifiers(mocker):
    mocker.patch("app.services.code_generator.ros_modifiers.update_setup_py")
    mocker.patch("app.services.code_generator.ros_modifiers.update_package_xml")
    # Mock other ros_modifiers functions if needed for tests
    return mocker.patch("app.services.code_generator.ros_modifiers")

@pytest.fixture
def mock_os_calls(mocker):
    mock_makedirs = mocker.patch("os.makedirs")
    mock_exists = mocker.patch("os.path.exists", return_value=False)
    mock_chmod = mocker.patch("os.chmod")
    m_open = mock_open()
    mocker.patch("builtins.open", m_open)
    return {
        "makedirs": mock_makedirs,
        "exists": mock_exists,
        "chmod": mock_chmod,
        "open": m_open
    }

@pytest.fixture
def mock_subprocess_run(mocker):
    mock_result = MagicMock()
    mock_result.stdout = "Colcon build successful"
    mock_result.stderr = ""
    mock_result.check_returncode.return_value = None # Simulate success
    run_mock = mocker.patch("subprocess.run", return_value=mock_result)
    return run_mock

@pytest.mark.asyncio
async def test_generate_pub_sub_success(mocker, mock_settings, mock_ros_modifiers, mock_os_calls, mock_subprocess_run):
    """Test the successful flow of generate_pub_sub."""
    request = UploadRequest(
        file_name="my_node.py",
        code="print('hello world')",
        type="pub_sub"
    )
    node_name = "my_node"
    scripts_dir = os.path.join("/fake", "scripts", "dir")
    file_path = os.path.join(scripts_dir, request.file_name)
    init_path = os.path.join(scripts_dir, "__init__.py")
    setup_py_path = "/ros2_ws/src/sample_pkg/setup.py"
    package_xml_path = "/ros2_ws/src/sample_pkg/package.xml"

    # Mock logging to avoid output
    mocker.patch("app.services.code_generator.logging")

    response = await code_generator.generate_pub_sub(request)

    # Verifications
    mock_os_calls["makedirs"].assert_called_once_with(scripts_dir, exist_ok=True)
    # Verify writing of node and __init__.py
    assert mock_os_calls["open"].call_count == 2
    mock_os_calls["open"].assert_any_call(file_path, "w")
    mock_os_calls["open"].assert_any_call(init_path, "w")
    handle = mock_os_calls["open"]()
    handle.write.assert_any_call(request.code) # Verify node content
    handle.write.assert_any_call("")          # Verify __init__.py content
    
    mock_os_calls["chmod"].assert_called_once_with(file_path, 0o755)
    mock_ros_modifiers.update_setup_py.assert_called_once_with(setup_py_path, node_name, request.type)
    mock_ros_modifiers.update_package_xml.assert_called_once_with(package_xml_path)
    mock_subprocess_run.assert_called_once_with(
        "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --symlink-install'",
        shell=True, check=False, capture_output=True, text=True
    )

    assert isinstance(response, JSONResponse)
    assert response.status_code == 200
    assert json.loads(response.body.decode()) == {"message": "File uploaded and package rebuilt", "file": request.file_name}

@pytest.mark.asyncio
async def test_generate_pub_sub_init_exists(mocker, mock_settings, mock_ros_modifiers, mock_os_calls, mock_subprocess_run):
    """Test generate_pub_sub when __init__.py already exists."""
    request = UploadRequest(file_name="my_node.py", code="", type="pub_sub")
    scripts_dir = os.path.join("/fake", "scripts", "dir")
    file_path = os.path.join(scripts_dir, request.file_name)
    init_path = os.path.join(scripts_dir, "__init__.py")
    
    # Modify the existing mock from the fixture, DO NOT repatch
    mock_os_calls["exists"].return_value = True 

    mocker.patch("app.services.code_generator.logging")
    
    await code_generator.generate_pub_sub(request)

    # Verify that __init__.py was NOT attempted to be opened for writing
    calls = mock_os_calls["open"].call_args_list
    # It should have been called only once (for the node file)
    assert len(calls) == 1 
    assert calls[0].args[0] == file_path
    assert calls[0].args[1] == "w"
    # Verify that exists was called with init_path, ignoring other calls
    mock_os_calls["exists"].assert_any_call(init_path)

@pytest.mark.asyncio
async def test_generate_pub_sub_colcon_fails(mocker, mock_settings, mock_ros_modifiers, mock_os_calls, mock_subprocess_run):
    """Test generate_pub_sub when colcon build fails."""
    request = UploadRequest(file_name="my_node.py", code="", type="pub_sub")

    # Simulate failure in subprocess
    mock_result = MagicMock()
    mock_result.stdout = "Build started"
    mock_result.stderr = "Error during build"
    # Change the mock to raise an exception or have a stderr
    mock_subprocess_run.return_value = mock_result 
    # Optionally, you could make it raise CalledProcessError if check=True was used
    # mock_subprocess_run.side_effect = subprocess.CalledProcessError(1, "cmd", stderr="Error")

    mocker.patch("app.services.code_generator.logging")
    
    response = await code_generator.generate_pub_sub(request)

    # Verify that colcon execution was attempted
    mock_subprocess_run.assert_called_once()
    
    # Verify the error response (although the current function does not propagate the colcon error explicitly as 500)
    # The current function simply logs the error and returns 200. Adjust according to desired behavior.
    assert isinstance(response, JSONResponse)
    assert response.status_code == 200 # The current function does not return 500 on colcon failure
    assert json.loads(response.body.decode()) == {"message": "File uploaded and package rebuilt", "file": request.file_name}