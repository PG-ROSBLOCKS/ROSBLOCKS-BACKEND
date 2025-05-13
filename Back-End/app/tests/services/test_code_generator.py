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

@pytest.mark.asyncio
async def test_generate_pub_sub_makedirs_fails(mocker, mock_settings, mock_ros_modifiers, mock_os_calls):
    """Test generate_pub_sub when os.makedirs raises an exception."""
    request = UploadRequest(file_name="my_node.py", code="print('hello')", type="pub_sub")
    scripts_dir = code_generator.settings.SCRIPTS_DIR

    # Configure os.makedirs to fail (part of mock_os_calls fixture)
    mock_os_calls["makedirs"].side_effect = OSError("Failed to create directory")
    
    mock_log_error = mocker.patch("app.services.code_generator.logging.error")

    response = await code_generator.generate_pub_sub(request)

    mock_os_calls["makedirs"].assert_called_once_with(scripts_dir, exist_ok=True)
    mock_log_error.assert_called_once_with(f"Unexpected error: Failed to create directory")

    assert isinstance(response, JSONResponse)
    assert response.status_code == 500
    content = json.loads(response.body.decode())
    assert content == {"error": "Unexpected error", "details": "Failed to create directory"}

@pytest.mark.asyncio
async def test_generate_service_success(mocker, mock_settings, mock_ros_modifiers, mock_os_calls, mock_subprocess_run):
    """Test the successful flow of generate_service."""
    request = UploadRequest(
        file_name="MyService.srv",
        code="int64 a\n---\nint64 b",
        type="srv" # type is not directly used by generate_service but good for consistency
    )
    service_name = "MyService" # Derived from file_name
    # Access settings via the code_generator module where it's patched
    srv_dir = os.path.join(code_generator.settings.INTERFACES_DIR, "srv")
    file_path = os.path.join(srv_dir, request.file_name)
    package_xml_path = os.path.join(code_generator.settings.INTERFACES_DIR, "package.xml")
    cmake_lists_path = os.path.join(code_generator.settings.INTERFACES_DIR, "CMakeLists.txt")

    # Mock logging to avoid output
    mocker.patch("app.services.code_generator.logging")

    response = await code_generator.generate_service(request)

    # Verifications
    mock_os_calls["makedirs"].assert_called_once_with(srv_dir, exist_ok=True)
    # Verify writing of the service file
    mock_os_calls["open"].assert_called_once_with(file_path, "w")
    handle = mock_os_calls["open"]()
    handle.write.assert_called_once_with(request.code) # Verify service file content
    
    # Verify ros_modifiers calls
    # Access the correctly patched object via the mock_ros_modifiers fixture
    mock_ros_modifiers.update_package_xml_services.assert_called_once_with(package_xml_path)
    mock_ros_modifiers.update_cmake_lists_services.assert_called_once_with(cmake_lists_path, service_name)
    
    # Verify colcon build command
    mock_subprocess_run.assert_called_once_with(
        "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --packages-select sample_interfaces'",
        shell=True, check=False, capture_output=True, text=True
    )

    assert isinstance(response, JSONResponse)
    assert response.status_code == 200
    content = json.loads(response.body.decode())
    assert content == {"message": "Service uploaded and package rebuilt", "file": request.file_name}

@pytest.mark.asyncio
async def test_generate_service_colcon_fails(mocker, mock_settings, mock_ros_modifiers, mock_os_calls, mock_subprocess_run):
    """Test generate_service when colcon build fails."""
    request = UploadRequest(
        file_name="FailService.srv",
        code="int64 a\n---\nint64 b",
        type="srv"
    )
    service_name = "FailService"
    srv_dir = os.path.join(code_generator.settings.INTERFACES_DIR, "srv")
    file_path = os.path.join(srv_dir, request.file_name)
    package_xml_path = os.path.join(code_generator.settings.INTERFACES_DIR, "package.xml")
    cmake_lists_path = os.path.join(code_generator.settings.INTERFACES_DIR, "CMakeLists.txt")

    # Simulate failure in subprocess for colcon build
    mock_colcon_result = MagicMock()
    mock_colcon_result.stdout = "Build started for interfaces"
    mock_colcon_result.stderr = "Error during interface build"
    mock_subprocess_run.return_value = mock_colcon_result

    mocker.patch("app.services.code_generator.logging.info") # Mock info to check specific calls if needed
    mock_log_error = mocker.patch("app.services.code_generator.logging.error")

    response = await code_generator.generate_service(request)

    # Basic verifications (file creation and ros_modifier calls should still happen)
    mock_os_calls["makedirs"].assert_called_once_with(srv_dir, exist_ok=True)
    mock_os_calls["open"].assert_called_once_with(file_path, "w")
    mock_ros_modifiers.update_package_xml_services.assert_called_once_with(package_xml_path)
    mock_ros_modifiers.update_cmake_lists_services.assert_called_once_with(cmake_lists_path, service_name)

    # Verify colcon build was attempted
    mock_subprocess_run.assert_called_once_with(
        "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --packages-select sample_interfaces'",
        shell=True, check=False, capture_output=True, text=True
    )
    # Verify the error from colcon was logged
    mock_log_error.assert_any_call(f"Colcon build error:\n{mock_colcon_result.stderr}")

    # The function currently returns 200 even on colcon build error, just logs it.
    assert isinstance(response, JSONResponse)
    assert response.status_code == 200 
    content = json.loads(response.body.decode())
    assert content == {"message": "Service uploaded and package rebuilt", "file": request.file_name}

@pytest.mark.asyncio
async def test_generate_message_success(mocker, mock_settings, mock_ros_modifiers, mock_os_calls, mock_subprocess_run):
    """Test the successful flow of generate_message."""
    request = UploadRequest(
        file_name="MyMessage.msg",
        code="string data\nint32 value",
        type="msg" 
    )
    message_name = "MyMessage" 
    msg_dir = os.path.join(code_generator.settings.INTERFACES_DIR, "msg")
    file_path = os.path.join(msg_dir, request.file_name)
    package_xml_path = os.path.join(code_generator.settings.INTERFACES_DIR, "package.xml")
    cmake_lists_path = os.path.join(code_generator.settings.INTERFACES_DIR, "CMakeLists.txt")

    mocker.patch("app.services.code_generator.logging")

    response = await code_generator.generate_message(request)

    mock_os_calls["makedirs"].assert_called_once_with(msg_dir, exist_ok=True)
    mock_os_calls["open"].assert_called_once_with(file_path, "w")
    handle = mock_os_calls["open"]()
    handle.write.assert_called_once_with(request.code)
    
    mock_ros_modifiers.update_package_xml_services.assert_called_once_with(package_xml_path) # Assuming this is general for interfaces
    mock_ros_modifiers.update_cmake_lists_messages.assert_called_once_with(cmake_lists_path, message_name)
    
    mock_subprocess_run.assert_called_once_with(
        "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --packages-select sample_interfaces'",
        shell=True, check=False, capture_output=True, text=True
    )

    assert isinstance(response, JSONResponse)
    assert response.status_code == 200
    content = json.loads(response.body.decode())
    assert content == {"message": "Msg uploaded and package rebuilt", "file": request.file_name} # Note: Message might differ

@pytest.mark.asyncio
async def test_generate_message_colcon_fails(mocker, mock_settings, mock_ros_modifiers, mock_os_calls, mock_subprocess_run):
    """Test generate_message when colcon build fails."""
    request = UploadRequest(
        file_name="FailMessage.msg",
        code="string data",
        type="msg"
    )
    message_name = "FailMessage"
    msg_dir = os.path.join(code_generator.settings.INTERFACES_DIR, "msg")
    file_path = os.path.join(msg_dir, request.file_name)
    package_xml_path = os.path.join(code_generator.settings.INTERFACES_DIR, "package.xml")
    cmake_lists_path = os.path.join(code_generator.settings.INTERFACES_DIR, "CMakeLists.txt")

    mock_colcon_result = MagicMock()
    mock_colcon_result.stdout = "Build started for interfaces msg"
    mock_colcon_result.stderr = "Error during interface msg build"
    mock_subprocess_run.return_value = mock_colcon_result

    mocker.patch("app.services.code_generator.logging.info")
    mock_log_error = mocker.patch("app.services.code_generator.logging.error")

    response = await code_generator.generate_message(request)

    mock_os_calls["makedirs"].assert_called_once_with(msg_dir, exist_ok=True)
    mock_os_calls["open"].assert_called_once_with(file_path, "w")
    mock_ros_modifiers.update_package_xml_services.assert_called_once_with(package_xml_path)
    mock_ros_modifiers.update_cmake_lists_messages.assert_called_once_with(cmake_lists_path, message_name)

    mock_subprocess_run.assert_called_once_with(
        "bash -c 'source /ros2_ws/install/setup.bash && cd /ros2_ws && colcon build --packages-select sample_interfaces'",
        shell=True, check=False, capture_output=True, text=True
    )
    mock_log_error.assert_any_call(f"Colcon build error:\n{mock_colcon_result.stderr}")

    assert isinstance(response, JSONResponse)
    assert response.status_code == 200
    content = json.loads(response.body.decode())
    assert content == {"message": "Msg uploaded and package rebuilt", "file": request.file_name}

@pytest.mark.asyncio
async def test_generate_server_success(mocker, mock_settings, mock_ros_modifiers, mock_os_calls, mock_subprocess_run):
    """Test the successful flow of generate_server."""
    request = UploadRequest(
        file_name="my_server_node.py",
        code="print('hello server')",
        type="server_node" 
    )
    node_name = "my_server_node"
    scripts_dir = code_generator.settings.SCRIPTS_DIR
    file_path = os.path.join(scripts_dir, request.file_name)
    init_path = os.path.join(scripts_dir, "__init__.py")
    setup_py_path = "/ros2_ws/src/sample_pkg/setup.py"
    package_xml_path = "/ros2_ws/src/sample_pkg/package.xml"

    mocker.patch("app.services.code_generator.logging")
    mock_os_calls["exists"].return_value = False # Ensure __init__.py is created

    response = await code_generator.generate_server(request)

    mock_os_calls["makedirs"].assert_called_once_with(scripts_dir, exist_ok=True)
    assert mock_os_calls["open"].call_count == 2
    mock_os_calls["open"].assert_any_call(file_path, "w")
    mock_os_calls["open"].assert_any_call(init_path, "w")
    handle = mock_os_calls["open"]()
    handle.write.assert_any_call(request.code)
    handle.write.assert_any_call("") # For __init__.py
    
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
async def test_generate_server_colcon_fails(mocker, mock_settings, mock_ros_modifiers, mock_os_calls, mock_subprocess_run):
    """Test generate_server when colcon build fails."""
    request = UploadRequest(file_name="fail_server.py", code="", type="server_node")
    scripts_dir = code_generator.settings.SCRIPTS_DIR
    
    mock_colcon_result = MagicMock()
    mock_colcon_result.stdout = "Build started"
    mock_colcon_result.stderr = "Error during server build"
    mock_subprocess_run.return_value = mock_colcon_result

    mock_log_error = mocker.patch("app.services.code_generator.logging.error")
    mocker.patch("app.services.code_generator.logging.info")
    mock_os_calls["exists"].return_value = True # Assume __init__.py exists

    response = await code_generator.generate_server(request)
    
    mock_subprocess_run.assert_called_once()
    mock_log_error.assert_any_call(f"Colcon build error:\n{mock_colcon_result.stderr}")
    
    assert isinstance(response, JSONResponse)
    assert response.status_code == 200 
    assert json.loads(response.body.decode()) == {"message": "File uploaded and package rebuilt", "file": request.file_name}

@pytest.mark.asyncio
async def test_generate_client_success(mocker, mock_settings, mock_ros_modifiers, mock_os_calls, mock_subprocess_run):
    """Test the successful flow of generate_client."""
    request = UploadRequest(
        file_name="my_client_node.py",
        code="print('hello client')",
        type="client_node" 
    )
    node_name = "my_client_node"
    scripts_dir = code_generator.settings.SCRIPTS_DIR
    file_path = os.path.join(scripts_dir, request.file_name)
    init_path = os.path.join(scripts_dir, "__init__.py")
    setup_py_path = "/ros2_ws/src/sample_pkg/setup.py"
    package_xml_path = "/ros2_ws/src/sample_pkg/package.xml"

    mocker.patch("app.services.code_generator.logging")
    mock_os_calls["exists"].return_value = False # Ensure __init__.py is created

    response = await code_generator.generate_client(request)

    mock_os_calls["makedirs"].assert_called_once_with(scripts_dir, exist_ok=True)
    assert mock_os_calls["open"].call_count == 2
    mock_os_calls["open"].assert_any_call(file_path, "w")
    mock_os_calls["open"].assert_any_call(init_path, "w")
    handle = mock_os_calls["open"]()
    handle.write.assert_any_call(request.code)
    handle.write.assert_any_call("") # For __init__.py
    
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
async def test_generate_client_colcon_fails(mocker, mock_settings, mock_ros_modifiers, mock_os_calls, mock_subprocess_run):
    """Test generate_client when colcon build fails."""
    request = UploadRequest(file_name="fail_client.py", code="", type="client_node")
    scripts_dir = code_generator.settings.SCRIPTS_DIR
    
    mock_colcon_result = MagicMock()
    mock_colcon_result.stdout = "Build started"
    mock_colcon_result.stderr = "Error during client build"
    mock_subprocess_run.return_value = mock_colcon_result

    mock_log_error = mocker.patch("app.services.code_generator.logging.error")
    mocker.patch("app.services.code_generator.logging.info")
    mock_os_calls["exists"].return_value = True # Assume __init__.py exists

    response = await code_generator.generate_client(request)
    
    mock_subprocess_run.assert_called_once()
    mock_log_error.assert_any_call(f"Colcon build error:\n{mock_colcon_result.stderr}")
    
    assert isinstance(response, JSONResponse)
    assert response.status_code == 200 
    assert json.loads(response.body.decode()) == {"message": "File uploaded and package rebuilt", "file": request.file_name}