# app/utils/ros_modifiers.py
import xml.etree.ElementTree as ET
import logging, re

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)


def update_setup_py(setup_file: str, node_name: str, requestType: str = "none", remove: bool = False):
    with open(setup_file, "r") as f:
        lines = f.readlines()
    entry_point_line = f'            "{node_name} = sample_pkg.{node_name}:main",\n'
    service_entry_point_line = f'            "service = py_srvcli.service_member_function:main,\n'

    inside_entry_points = False
    new_lines = []
    entry_found = False
    for line in lines:
        if "'console_scripts': [" in line:
            inside_entry_points = True
        if inside_entry_points and entry_point_line.strip() in line.strip():
            entry_found = True
            if remove:
                continue
        if inside_entry_points and "]" in line:
            if not entry_found and not remove:
                new_lines.append(entry_point_line)
            if requestType.startswith("service") and not remove:
                new_lines.append(service_entry_point_line)
            inside_entry_points = False
        new_lines.append(line)
    with open(setup_file, "w") as f:
        f.writelines(new_lines)
    action = "Eliminado" if remove else "Agregado"
    print(f"{action} {node_name} en setup.py")


def update_package_xml(package_xml_file: str, remove: bool = False):
    tree = ET.parse(package_xml_file)
    root = tree.getroot()
    dependencies = ["rclpy", "std_msgs"]
    if remove:
        for dep in dependencies:
            for child in root.findall("depend"):
                if child.text == dep:
                    root.remove(child)
    else:
        for dep in dependencies:
            if not any(child.text == dep for child in root.findall("depend")):
                new_depend = ET.Element("depend")
                new_depend.text = dep
                root.append(new_depend)
    tree.write(package_xml_file)
    action = "Eliminadas" if remove else "Agregadas"
    print(f"{action} dependencias en package.xml")


def update_package_xml_services(package_xml_file: str, dependencies: list = []):
    tree = ET.parse(package_xml_file)
    root = tree.getroot()
    required_dependencies = [
        ("buildtool_depend", "rosidl_default_generators"),
        ("exec_depend", "rosidl_default_runtime"),
        ("member_of_group", "rosidl_interface_packages"),
    ]
    for tag, dep in required_dependencies:
        if not any(child.text == dep for child in root.findall(tag)):
            new_element = ET.Element(tag)
            new_element.text = dep
            root.append(new_element)
    for dep in dependencies:
        if not any(child.text == dep for child in root.findall("depend")):
            new_depend = ET.Element("depend")
            new_depend.text = dep
            root.append(new_depend)
    tree.write(package_xml_file)
    print(f"Actualizado package.xml con dependencias: {dependencies + [d[1] for d in required_dependencies]}")


def update_cmake_lists_services(cmake_file: str, service_name: str, dependencies: list = [], remove: bool = False):
    """
    Si remove es False, se agrega la línea para "srv/{service_name}.srv" dentro de
    rosidl_generate_interfaces. Si remove es True, se elimina la línea correspondiente.
    En modo eliminación, si no se encuentra la entrada, se lanza una excepción.
    """
    with open(cmake_file, "r") as f:
        lines = f.readlines()
    ament_cmake_index = next((i for i, line in enumerate(lines) if "find_package(ament_cmake REQUIRED)" in line), None)
    if ament_cmake_index is None:
        raise ValueError("find_package(ament_cmake REQUIRED) no encontrado en CMakeLists.txt.")
    if not any("find_package(rosidl_default_generators REQUIRED)" in line for line in lines):
        lines.insert(ament_cmake_index + 1, "find_package(rosidl_default_generators REQUIRED)\n")
    dependencies = list(set(dependencies + ["std_msgs"]))
    for dep in dependencies:
        find_package_line = f"find_package({dep} REQUIRED)\n"
        if not any(f"find_package({dep} REQUIRED)" in line for line in lines):
            lines.insert(ament_cmake_index + 2, find_package_line)
    rosidl_start_index = next((i for i, line in enumerate(lines) if "rosidl_generate_interfaces(" in line), None)
    rosidl_end_index = None
    dependencies_line_index = None
    if rosidl_start_index is not None:
        for i in range(rosidl_start_index, len(lines)):
            if "DEPENDENCIES" in lines[i]:
                dependencies_line_index = i
            if ")" in lines[i]:
                rosidl_end_index = i
                break

        # Filtrar las entradas de servicios existentes
        found_entry = False
        new_rosidl_block_lines = []
        for i in range(rosidl_start_index, rosidl_end_index):
            if re.search(r'\s*"srv/{}\.srv"'.format(re.escape(service_name)), lines[i]):
                found_entry = True
                if remove:
                    # Si se desea eliminar, se omite esta línea
                    continue
            new_rosidl_block_lines.append(lines[i])
        # En modo eliminación, si no se encontró la entrada, se lanza una excepción
        if remove and not found_entry:
            raise ValueError(f"No se encontró la entrada 'srv/{service_name}.srv' en CMakeLists.txt.")
        # Si no es remove y la entrada no está presente, agregarla
        if not remove:
            service_present = any(re.search(r'\s*"srv/{}\.srv"'.format(re.escape(service_name)), line) for line in new_rosidl_block_lines)
            if not service_present:
                insert_index = dependencies_line_index if dependencies_line_index is not None else rosidl_end_index
                new_rosidl_block_lines.insert(insert_index - rosidl_start_index, f'  "srv/{service_name}.srv"\n')
        # Reconstruir la sección, removiendo cualquier línea DEPENDENCIES
        new_rosidl_block_lines = [line for line in new_rosidl_block_lines if not re.search(r"\s*DEPENDENCIES\s+", line)]
        new_rosidl_block_lines.append(f"  DEPENDENCIES {' '.join(sorted(dependencies))}\n")
        lines = lines[:rosidl_start_index] + new_rosidl_block_lines + lines[rosidl_end_index:]
    else:
        ament_package_index = next((i for i, line in enumerate(lines) if "ament_package()" in line), None)
        if ament_package_index is None:
            raise ValueError("ament_package() no encontrado en CMakeLists.txt.")
        if remove:
            raise ValueError(f"No se encontró sección rosidl_generate_interfaces, nada que eliminar para {service_name}.srv")
        rosidl_block = f"""
rosidl_generate_interfaces(${{PROJECT_NAME}}
  "srv/{service_name}.srv"
  DEPENDENCIES {" ".join(sorted(dependencies))}
)\n"""
        lines.insert(ament_package_index, rosidl_block)
    with open(cmake_file, "w") as f:
        f.writelines(lines)
    action = "Eliminado" if remove else "Agregado"
    print(f"CMakeLists.txt actualizado: {action} `{service_name}.srv` en rosidl_generate_interfaces.")


def update_cmake_lists_messages(cmake_file: str, message_name: str, dependencies: list = [], remove: bool = False):
    """
    Si remove es False, se agrega la línea para "msg/{message_name}.msg" dentro de
    rosidl_generate_interfaces. Si remove es True, se elimina la línea correspondiente.
    En modo eliminación, si no se encuentra la entrada, se lanza una excepción.
    """
    with open(cmake_file, "r") as f:
        lines = f.readlines()
    ament_cmake_index = next((i for i, line in enumerate(lines) if "find_package(ament_cmake REQUIRED)" in line), None)
    if ament_cmake_index is None:
        raise ValueError("find_package(ament_cmake REQUIRED) no encontrado en CMakeLists.txt.")
    if not any("find_package(rosidl_default_generators REQUIRED)" in line for line in lines):
        lines.insert(ament_cmake_index + 1, "find_package(rosidl_default_generators REQUIRED)\n")
    dependencies = list(set(dependencies + ["std_msgs"]))
    for dep in dependencies:
        find_package_line = f"find_package({dep} REQUIRED)\n"
        if not any(f"find_package({dep} REQUIRED)" in line for line in lines):
            lines.insert(ament_cmake_index + 2, find_package_line)
    rosidl_start_index = next((i for i, line in enumerate(lines) if "rosidl_generate_interfaces(" in line), None)
    rosidl_end_index = None
    dependencies_line_index = None
    if rosidl_start_index is not None:
        for i in range(rosidl_start_index, len(lines)):
            if "DEPENDENCIES" in lines[i]:
                dependencies_line_index = i
            if ")" in lines[i]:
                rosidl_end_index = i
                break

        found_entry = False
        new_rosidl_block_lines = []
        for i in range(rosidl_start_index, rosidl_end_index):
            if re.search(r'\s*"msg/{}\.msg"'.format(re.escape(message_name)), lines[i]):
                found_entry = True
                if remove:
                    continue
            new_rosidl_block_lines.append(lines[i])
        if remove and not found_entry:
            raise ValueError(f"No se encontró la entrada 'msg/{message_name}.msg' en CMakeLists.txt.")
        if not remove:
            message_present = any(re.search(r'\s*"msg/{}\.msg"'.format(re.escape(message_name)), line) for line in new_rosidl_block_lines)
            if not message_present:
                insert_index = dependencies_line_index if dependencies_line_index is not None else rosidl_end_index
                new_rosidl_block_lines.insert(insert_index - rosidl_start_index, f'  "msg/{message_name}.msg"\n')
        new_rosidl_block_lines = [line for line in new_rosidl_block_lines if not re.search(r"\s*DEPENDENCIES\s+", line)]
        new_rosidl_block_lines.append(f"  DEPENDENCIES {' '.join(sorted(dependencies))}\n")
        lines = lines[:rosidl_start_index] + new_rosidl_block_lines + lines[rosidl_end_index:]
    else:
        ament_package_index = next((i for i, line in enumerate(lines) if "ament_package()" in line), None)
        if ament_package_index is None:
            raise ValueError("ament_package() no encontrado en CMakeLists.txt.")
        if remove:
            raise ValueError(f"No se encontró sección rosidl_generate_interfaces, nada que eliminar para {message_name}.msg")
        rosidl_block = f"""
rosidl_generate_interfaces(${{PROJECT_NAME}}
  "msg/{message_name}.msg"
  DEPENDENCIES {" ".join(sorted(dependencies))}
)\n"""
        lines.insert(ament_package_index, rosidl_block)
    with open(cmake_file, "w") as f:
        f.writelines(lines)
    action = "Eliminado" if remove else "Agregado"
    print(f"CMakeLists.txt actualizado: {action} `{message_name}.msg` en rosidl_generate_interfaces.")
