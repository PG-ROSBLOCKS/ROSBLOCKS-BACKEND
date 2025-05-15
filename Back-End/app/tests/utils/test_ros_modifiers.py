# Back-End/app/tests/utils/test_ros_modifiers.py
import pytest
from unittest.mock import patch, mock_open
from app.utils import ros_modifiers
import xml.etree.ElementTree as ET
from unittest.mock import MagicMock

# --- Tests for update_setup_py ---

@pytest.fixture
def setup_py_content_base():
    """Base content of setup.py for tests."""
    return """
from setuptools import setup

package_name = 'sample_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Existing entries can go here if needed for tests
        ],
    },
)
"""

@pytest.fixture
def setup_py_content_with_node():
    """Content of setup.py with an existing node."""
    return """
from setuptools import setup

package_name = 'sample_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "existing_node = sample_pkg.existing_node:main",
        ],
    },
)
"""

def test_update_setup_py_add_new_node(mocker, setup_py_content_base):
    """Test adding a new node to an empty setup.py."""
    mock_file = mock_open(read_data=setup_py_content_base)
    mocker.patch("builtins.open", mock_file)
    mocker.patch("builtins.print")
    ros_modifiers.update_setup_py("dummy_setup.py", "new_node")
    mock_file.assert_called_with("dummy_setup.py", "w")
    handle = mock_file()
    handle.writelines.assert_called_once()
    written_lines = handle.writelines.call_args[0][0]
    written_content = "".join(written_lines)
    assert '"new_node = sample_pkg.new_node:main",' in written_content
    assert "'console_scripts': [" in written_content


def test_update_setup_py_add_existing_node(mocker, setup_py_content_with_node):
    """Test adding an existing node (should not duplicate)."""
    mock_file = mock_open(read_data=setup_py_content_with_node)
    mocker.patch("builtins.open", mock_file)
    mocker.patch("builtins.print")
    ros_modifiers.update_setup_py("dummy_setup.py", "existing_node")
    handle = mock_file()
    handle.writelines.assert_called_once()
    written_lines = handle.writelines.call_args[0][0]
    written_content = "".join(written_lines)
    occurrences = written_content.count('"existing_node = sample_pkg.existing_node:main",')
    assert occurrences == 1


def test_update_setup_py_remove_existing_node(mocker, setup_py_content_with_node):
    """Test remove an existing node."""
    mock_file = mock_open(read_data=setup_py_content_with_node)
    mocker.patch("builtins.open", mock_file)
    mocker.patch("builtins.print")
    ros_modifiers.update_setup_py("dummy_setup.py", "existing_node", remove=True)
    handle = mock_file()
    handle.writelines.assert_called_once()
    written_lines = handle.writelines.call_args[0][0]
    written_content = "".join(written_lines)
    assert '"existing_node = sample_pkg.existing_node:main",' not in written_content


def test_update_setup_py_remove_nonexistent_node(mocker, setup_py_content_base):
    """Test remove a nonexistent node (should not change anything)."""
    mock_file = mock_open(read_data=setup_py_content_base)
    mocker.patch("builtins.open", mock_file)
    mocker.patch("builtins.print")
    ros_modifiers.update_setup_py("dummy_setup.py", "nonexistent_node", remove=True)
    handle = mock_file()
    handle.writelines.assert_called_once()
    written_lines = handle.writelines.call_args[0][0]
    written_content = "".join(written_lines)
    assert written_content.strip() == setup_py_content_base.strip()
    assert '"nonexistent_node = sample_pkg.nonexistent_node:main",' not in written_content


# --- Tests for update_package_xml ---

@pytest.fixture
def package_xml_content_base():
    """Base content of package.xml without relevant dependencies."""
    return """<?xml version="1.0"?>
<package format="3">
  <name>sample_pkg</name>
  <version>0.0.0</version>
  <description>The sample_pkg package</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
"""

@pytest.fixture
def package_xml_content_with_deps():
    """Content of package.xml with rclpy already present."""
    return """<?xml version="1.0"?>
<package format="3">
  <name>sample_pkg</name>
  <version>0.0.0</version>
  <description>The sample_pkg package</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake_python</buildtool_depend>
  <depend>rclpy</depend> 

  <test_depend>ament_copyright</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
"""

# Helper to verify dependencies in the mocked XML tree
def check_deps_in_mock_tree(mock_tree, expected_deps):
    root = mock_tree.getroot()
    current_deps = {elem.text for elem in root.findall("depend")}
    return current_deps == set(expected_deps)

def test_update_package_xml_add_defaults(mocker, package_xml_content_base):
    """Test add the default dependencies (rclpy, std_msgs)."""
    # Mock ET.parse to return a mock tree based on the string
    mock_tree = ET.ElementTree(ET.fromstring(package_xml_content_base))
    mock_parse = mocker.patch("xml.etree.ElementTree.parse", return_value=mock_tree)
    # Mock tree.write to capture the call without writing the file
    mock_write = mocker.patch.object(mock_tree, "write")
    mocker.patch("builtins.print")

    ros_modifiers.update_package_xml("dummy_package.xml")

    mock_parse.assert_called_once_with("dummy_package.xml")
    mock_write.assert_called_once_with("dummy_package.xml")

    # Verify that the dependencies are in the modified tree
    assert check_deps_in_mock_tree(mock_tree, ["rclpy", "std_msgs"])


def test_update_package_xml_add_with_existing(mocker, package_xml_content_with_deps):
    """Test add dependencies when one already exists (should not duplicate)."""
    mock_tree = ET.ElementTree(ET.fromstring(package_xml_content_with_deps))
    mock_parse = mocker.patch("xml.etree.ElementTree.parse", return_value=mock_tree)
    mock_write = mocker.patch.object(mock_tree, "write")
    mocker.patch("builtins.print")

    ros_modifiers.update_package_xml("dummy_package.xml")

    assert check_deps_in_mock_tree(mock_tree, ["rclpy", "std_msgs"])


def test_update_package_xml_remove_nonexistent(mocker, package_xml_content_base):
    """Test remove dependencies when they do not exist (should not fail)."""
    mock_tree = ET.ElementTree(ET.fromstring(package_xml_content_base))
    mock_parse = mocker.patch("xml.etree.ElementTree.parse", return_value=mock_tree)
    mock_write = mocker.patch.object(mock_tree, "write")
    mocker.patch("builtins.print")

    # Verify that initially they are not
    assert check_deps_in_mock_tree(mock_tree, [])

    ros_modifiers.update_package_xml("dummy_package.xml", remove=True)

    # Verify that they are not (and no error)
    assert check_deps_in_mock_tree(mock_tree, [])
    mock_write.assert_called_once() # Ensure it attempted to write 

# --- Tests for update_cmake_lists_services ---

@pytest.fixture
def cmake_lists_content_base():
    """Base content of CMakeLists.txt for service tests."""
    return """\
cmake_minimum_required(VERSION 3.8)
project(sample_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Other dependencies can go here

rosidl_generate_interfaces(${PROJECT_NAME}
  # Existing .msg files can be listed here for context if needed
  # "msg/MyMessage.msg"
)

# install(DIRECTORY
#   include/${PROJECT_NAME}
#   DESTINATION include
# )

ament_export_dependencies(rosidl_default_runtime)
ament_package()
"""

@pytest.fixture
def cmake_lists_content_with_service():
    """Content of CMakeLists.txt with an existing service."""
    return """\
cmake_minimum_required(VERSION 3.8)
project(sample_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExistingService.srv"
  # "msg/MyMessage.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
"""

def test_update_cmake_lists_add_new_service(mocker, cmake_lists_content_base):
    """Test adding a new service to a base CMakeLists.txt."""
    mock_file = mock_open(read_data=cmake_lists_content_base)
    mocker.patch("builtins.open", mock_file)
    mocker.patch("builtins.print") # If your function prints
    
    ros_modifiers.update_cmake_lists_services("dummy_CMakeLists.txt", "NewService")
    
    mock_file.assert_called_with("dummy_CMakeLists.txt", "w")
    handle = mock_file()
    # Check that writelines was called (or write, depending on your implementation)
    handle.writelines.assert_called_once() 
    written_lines = handle.writelines.call_args[0][0]
    written_content = "".join(written_lines)
    
    # Verify the new service is in the rosidl_generate_interfaces block
    assert 'rosidl_generate_interfaces(${PROJECT_NAME}' in written_content
    assert '  "srv/NewService.srv"' in written_content
    # Ensure other parts are preserved
    assert "ament_package()" in written_content

# Placeholder for more tests
# def test_update_cmake_lists_add_to_existing_services(...):
# def test_update_cmake_lists_add_duplicate_service(...):
# def test_update_cmake_lists_remove_service(...):
# def test_update_cmake_lists_remove_nonexistent_service(...):

"" 