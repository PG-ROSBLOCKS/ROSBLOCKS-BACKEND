import xml.etree.ElementTree as ET  

def update_setup_py(setup_file, node_name):
    """ Agregar un nuevo ejecutable a setup.py """
    with open(setup_file, "r") as f:
        lines = f.readlines()

    entry_point_line = f'            "{node_name} = sample_pkg.{node_name}:main",\n'
    
    # Buscar la línea donde se definen los entry_points y asegurarse de que el nuevo nodo esté ahí
    inside_entry_points = False
    new_lines = []
    entry_found = False

    for line in lines:
        if "'console_scripts': [" in line:
            inside_entry_points = True

        if inside_entry_points and entry_point_line.strip() in line.strip():
            entry_found = True  # Ya está en setup.py

        if inside_entry_points and "]" in line:  # Cierra entry_points
            if not entry_found:
                new_lines.append(entry_point_line)  # Agregar nuevo nodo
            inside_entry_points = False

        new_lines.append(line)

    # Sobreescribir setup.py con los cambios
    with open(setup_file, "w") as f:
        f.writelines(new_lines)

def update_package_xml(package_xml_file):
    """ Asegurar que package.xml tenga las dependencias correctas """
    tree = ET.parse(package_xml_file)
    root = tree.getroot()

    dependencies = ["rclpy", "std_msgs"]

    for dep in dependencies:
        if not any(child.text == dep for child in root.findall("depend")):
            new_depend = ET.Element("depend")
            new_depend.text = dep
            root.append(new_depend)

    tree.write(package_xml_file)
