import xml.etree.ElementTree as ET  

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

