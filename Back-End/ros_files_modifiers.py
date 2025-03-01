import xml.etree.ElementTree as ET
import logging
import re

# Configurar logging
logging.basicConfig(
    level=logging.INFO,  # Nivel mínimo de logs que se mostrarán
    format="%(asctime)s - %(levelname)s - %(message)s",  # Formato del log
    datefmt="%Y-%m-%d %H:%M:%S"  # Formato de fecha y hora
)

def update_setup_py(setup_file: str, node_name: str, remove: bool = False):
    """ Agregar o eliminar un ejecutable en setup.py """
    with open(setup_file, "r") as f:
        lines = f.readlines()

    entry_point_line = f'            "{node_name} = sample_pkg.{node_name}:main",\n'

    inside_entry_points = False
    new_lines = []
    entry_found = False

    for line in lines:
        if "'console_scripts': [" in line:
            inside_entry_points = True

        if inside_entry_points and entry_point_line.strip() in line.strip():
            entry_found = True  # Ya está en setup.py
            if remove:
                continue  # 🗑️ Si estamos eliminando, saltamos la línea

        if inside_entry_points and "]" in line:
            if not entry_found and not remove:
                new_lines.append(entry_point_line)  # ➕ Agregar nodo si no existe
            inside_entry_points = False

        new_lines.append(line)

    # Sobreescribir setup.py con los cambios
    with open(setup_file, "w") as f:
        f.writelines(new_lines)

    action = "Eliminado" if remove else "Agregado"
    print(f"{action} {node_name} en setup.py")

import xml.etree.ElementTree as ET  

def update_package_xml(package_xml_file: str, remove: bool = False):
    """ Agregar o eliminar dependencias en package.xml """
    tree = ET.parse(package_xml_file)
    root = tree.getroot()

    dependencies = ["rclpy", "std_msgs"]

    if remove:
        # Eliminar dependencias si están presentes
        for dep in dependencies:
            for child in root.findall("depend"):
                if child.text == dep:
                    root.remove(child)
    else:
        # Agregar dependencias si no están
        for dep in dependencies:
            if not any(child.text == dep for child in root.findall("depend")):
                new_depend = ET.Element("depend")
                new_depend.text = dep
                root.append(new_depend)

    tree.write(package_xml_file)

    action = "Eliminadas" if remove else "Agregadas"
    print(f"{action} dependencias en package.xml")


def update_package_xml_services(package_xml_file: str, dependencies: list = []):
    """ Agrega dependencias necesarias en package.xml para la generación de servicios """
    
    tree = ET.parse(package_xml_file)
    root = tree.getroot()

    # Dependencias obligatorias para la generación de servicios
    required_dependencies = [
        ("buildtool_depend", "rosidl_default_generators"),
        ("exec_depend", "rosidl_default_runtime"),
        ("member_of_group", "rosidl_interface_packages"),
    ]

    # Agregar dependencias si no existen
    for tag, dep in required_dependencies:
        if not any(child.text == dep for child in root.findall(tag)):
            new_element = ET.Element(tag)
            new_element.text = dep
            root.append(new_element)

    # Agregar las dependencias del usuario en `<depend>`
    for dep in dependencies:
        if not any(child.text == dep for child in root.findall("depend")):
            new_depend = ET.Element("depend")
            new_depend.text = dep
            root.append(new_depend)

    # Guardar cambios en package.xml
    tree.write(package_xml_file)

    print(f"Actualizado package.xml con dependencias: {dependencies + [d[1] for d in required_dependencies]}")

def update_cmake_lists_services(cmake_file: str, service_name: str, dependencies: list = []):
    """Agrega un nuevo servicio .srv dentro del bloque rosidl_generate_interfaces en CMakeLists.txt de ROS 2."""
    
    with open(cmake_file, "r") as f:
        lines = f.readlines()

    # Buscar índice de `find_package(ament_cmake REQUIRED)`
    ament_cmake_index = next((i for i, line in enumerate(lines) if "find_package(ament_cmake REQUIRED)" in line), None)
    if ament_cmake_index is None:
        print("ERROR: `find_package(ament_cmake REQUIRED)` no encontrado en CMakeLists.txt.")
        return

    # Agregar `find_package(rosidl_default_generators REQUIRED)` si no está presente
    if not any("find_package(rosidl_default_generators REQUIRED)" in line for line in lines):
        lines.insert(ament_cmake_index + 1, "find_package(rosidl_default_generators REQUIRED)\n")

    # Lista de dependencias con `std_msgs` por defecto
    dependencies = list(set(dependencies + ["std_msgs"]))  # Evita duplicados, agrega `std_msgs` si no está

    # Agregar `find_package(...)` para cada dependencia si no están en el archivo
    for dep in dependencies:
        find_package_line = f"find_package({dep} REQUIRED)\n"
        if not any(f"find_package({dep} REQUIRED)" in line for line in lines):
            lines.insert(ament_cmake_index + 2, find_package_line)  # Insertar después de `ament_cmake` y `rosidl_default_generators`

    # Buscar si ya existe el bloque `rosidl_generate_interfaces`
    rosidl_start_index = next((i for i, line in enumerate(lines) if "rosidl_generate_interfaces(" in line), None)
    rosidl_end_index = None
    dependencies_line_index = None
    existing_services = set()

    if rosidl_start_index is not None:
        # Buscar el final del bloque `rosidl_generate_interfaces`
        for i in range(rosidl_start_index, len(lines)):
            if "DEPENDENCIES" in lines[i]:
                dependencies_line_index = i  # Guardar la posición de DEPENDENCIES
            if ")" in lines[i]:  # Fin del bloque
                rosidl_end_index = i
                break

        if rosidl_end_index is None:
            print("ERROR: Bloque `rosidl_generate_interfaces` no tiene cierre `)` en CMakeLists.txt.")
            return

        # Extraer servicios existentes para evitar duplicados
        for i in range(rosidl_start_index, rosidl_end_index):
            match = re.match(r'\s*"srv/(.+)\.srv"', lines[i])
            if match:
                existing_services.add(match.group(1))

        # Agregar nuevo servicio si no está ya presente
        if service_name not in existing_services:
            lines.insert(dependencies_line_index if dependencies_line_index else rosidl_end_index, f'  "srv/{service_name}.srv"\n')

        # Eliminar cualquier línea de DEPENDENCIES repetida antes de agregar la nueva
        lines = [line for line in lines if not re.search(r"\s*DEPENDENCIES\s+", line)]

        # Insertar la nueva línea de DEPENDENCIES al final del bloque
        lines.insert(rosidl_end_index, f"  DEPENDENCIES {' '.join(sorted(dependencies))}\n")

    else:
        # No existe el bloque, crearlo antes de `ament_package()`
        ament_package_index = next((i for i, line in enumerate(lines) if "ament_package()" in line), None)
        if ament_package_index is None:
            print("ERROR: `ament_package()` no encontrado en CMakeLists.txt.")
            return

        rosidl_block = f"""
rosidl_generate_interfaces(${{PROJECT_NAME}}
  "srv/{service_name}.srv"
  DEPENDENCIES {" ".join(sorted(dependencies))}
)\n"""
        lines.insert(ament_package_index, rosidl_block)

    # Guardar los cambios en el archivo
    with open(cmake_file, "w") as f:
        f.writelines(lines)

    print(f"CMakeLists.txt actualizado con `{service_name}.srv` dentro de `rosidl_generate_interfaces` sin duplicar DEPENDENCIES.")

def update_cmake_lists_messages(cmake_file: str, message_name: str, dependencies: list = []):
    """Agrega un nuevo mensaje .msg dentro del bloque rosidl_generate_interfaces en CMakeLists.txt de ROS 2."""
    
    with open(cmake_file, "r") as f:
        lines = f.readlines()

    # Buscar índice de `find_package(ament_cmake REQUIRED)`
    ament_cmake_index = next((i for i, line in enumerate(lines) if "find_package(ament_cmake REQUIRED)" in line), None)
    if ament_cmake_index is None:
        print("ERROR: `find_package(ament_cmake REQUIRED)` no encontrado en CMakeLists.txt.")
        return

    # Agregar `find_package(rosidl_default_generators REQUIRED)` si no está presente
    if not any("find_package(rosidl_default_generators REQUIRED)" in line for line in lines):
        lines.insert(ament_cmake_index + 1, "find_package(rosidl_default_generators REQUIRED)\n")

    # Lista de dependencias con `std_msgs` por defecto
    dependencies = list(set(dependencies + ["std_msgs"]))  # Evita duplicados, agrega `std_msgs` si no está

    # Agregar `find_package(...)` para cada dependencia si no están en el archivo
    for dep in dependencies:
        find_package_line = f"find_package({dep} REQUIRED)\n"
        if not any(f"find_package({dep} REQUIRED)" in line for line in lines):
            lines.insert(ament_cmake_index + 2, find_package_line)  # Insertar después de `ament_cmake` y `rosidl_default_generators`

    # Buscar si ya existe el bloque `rosidl_generate_interfaces`
    rosidl_start_index = next((i for i, line in enumerate(lines) if "rosidl_generate_interfaces(" in line), None)
    rosidl_end_index = None
    dependencies_line_index = None
    existing_messages = set()

    if rosidl_start_index is not None:
        # Buscar el final del bloque `rosidl_generate_interfaces`
        for i in range(rosidl_start_index, len(lines)):
            if "DEPENDENCIES" in lines[i]:
                dependencies_line_index = i  # Guardar la posición de DEPENDENCIES
            if ")" in lines[i]:  # Fin del bloque
                rosidl_end_index = i
                break

        if rosidl_end_index is None:
            print("ERROR: Bloque `rosidl_generate_interfaces` no tiene cierre `)` en CMakeLists.txt.")
            return

        # Extraer mensajes existentes para evitar duplicados
        for i in range(rosidl_start_index, rosidl_end_index):
            match = re.match(r'\s*"msg/(.+)\.msg"', lines[i])
            if match:
                existing_messages.add(match.group(1))

        # Agregar nuevo mensaje si no está ya presente
        if message_name not in existing_messages:
            lines.insert(dependencies_line_index if dependencies_line_index else rosidl_end_index, f'  "msg/{message_name}.msg"\n')

        # Eliminar cualquier línea de DEPENDENCIES repetida antes de agregar la nueva
        lines = [line for line in lines if not re.search(r"\s*DEPENDENCIES\s+", line)]

        # Insertar la nueva línea de DEPENDENCIES al final del bloque
        lines.insert(rosidl_end_index, f"  DEPENDENCIES {' '.join(sorted(dependencies))}\n")

    else:
        # No existe el bloque, crearlo antes de `ament_package()`
        ament_package_index = next((i for i, line in enumerate(lines) if "ament_package()" in line), None)
        if ament_package_index is None:
            print("ERROR: `ament_package()` no encontrado en CMakeLists.txt.")
            return

        rosidl_block = f"""
rosidl_generate_interfaces(${{PROJECT_NAME}}
  "msg/{message_name}.msg"
  DEPENDENCIES {" ".join(sorted(dependencies))}
)\n"""
        lines.insert(ament_package_index, rosidl_block)

    # Guardar los cambios en el archivo
    with open(cmake_file, "w") as f:
        f.writelines(lines)

    print(f"CMakeLists.txt actualizado con `{message_name}.msg` dentro de `rosidl_generate_interfaces` sin duplicar DEPENDENCIES.")
