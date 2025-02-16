import os
import sys

# Get an environment variable (returns None if not found)
raisin_ws = os.getenv("RAISIN_WS")
sys.path.append(raisin_ws)
from raisin_workspace_setup import *


script_directory = os.path.dirname(os.path.realpath(__file__))
package_template = os.path.join(script_directory, 'src', 'templates', 'package.xml')
cmakelists_template = os.path.join(script_directory, 'src', 'templates', 'CMakeLists.txt')
conversion_template = os.path.join(script_directory, 'src', 'templates', 'conversion.hpp')
script_directory = os.path.dirname(os.path.realpath(__file__))

def generate_package_xml(project_name, dependencies, destination_dir):
    with open(package_template, 'r') as template_file:
        package_content = template_file.read()
        
    package_content = package_content.replace('@@PROJECT_NAME@@', project_name)
    package_content = package_content.replace('@@DEPENDENCIES@@',  "\n  ".join(f"<depend>{dep}</depend>" for dep in dependencies))

    with open(os.path.join(destination_dir, 'package.xml'), 'w') as output_file:
        output_file.write(package_content)

def generate_cmakelists_txt(project_name, dependencies, destination_dir):
    with open(cmakelists_template, 'r') as template_file:
        cmakelists_content = template_file.read()
        
    cmakelists_content = cmakelists_content.replace('@@PROJECT_NAME@@', project_name)
    cmakelists_content = cmakelists_content.replace('@@FIND_DEPENDENCIES@@',  "\n".join(f"find_package({dep})" for dep in dependencies))

    with open(os.path.join(destination_dir, 'CMakeLists.txt'), 'w') as output_file:
        output_file.write(cmakelists_content)


def create_interface(destination_dir, project_directory):
    project_name = os.path.basename(project_directory)
    destination_dir = os.path.join(destination_dir, project_name)

    msg_dir = os.path.join(destination_dir, 'msg')
    srv_dir = os.path.join(destination_dir, 'srv')
    os.makedirs(msg_dir, exist_ok=True)
    os.makedirs(srv_dir, exist_ok=True)
    msg_files = find_msg_files(project_directory)
    srv_files = find_srv_files(project_directory)
    for msg_file in msg_files:
        shutil.copy2(msg_file, msg_dir)
    for srv_file in srv_files:
        shutil.copy2(srv_file, srv_dir)

    depencendies = ['geometry_msgs', 'std_msgs', 'sensor_msgs']
    generate_package_xml(project_name, depencendies, destination_dir)
    generate_cmakelists_txt(project_name, depencendies, destination_dir)

    with open(os.path.join(destination_dir, "..", "..", 'interfaces.hpp'), 'a') as output_file:
        for msg_file in msg_files:
            snake_str = re.sub(r'(?<!^)(?=[A-Z][a-z]|(?<=[a-z])[A-Z])', '_', os.path.splitext(os.path.basename(msg_file))[0]).lower()
            snake_str = snake_str.replace("__", "_")
            output_file.write('#include <' + project_name + '/msg/' + snake_str + '.hpp>\n')
        for srv_file in srv_files:
            snake_str = re.sub(r'(?<!^)(?=[A-Z][a-z]|(?<=[a-z])[A-Z])', '_', os.path.splitext(os.path.basename(srv_file))[0]).lower()
            snake_str = snake_str.replace("__", "_")
            output_file.write('#include <' + project_name + '/srv/' + snake_str + '.hpp>\n')


    with open(os.path.join(destination_dir, 'conversion.hpp'), 'a') as output_file:
        for msg_file in msg_files:
            pascal_str = os.path.splitext(os.path.basename(msg_file))[0]
            snake_str = re.sub(r'(?<!^)(?=[A-Z][a-z]|(?<=[a-z])[A-Z])', '_', pascal_str).lower()
            snake_str = snake_str.replace("__", "_")
            with open(conversion_template, 'r') as template_file:
                conversion_content = template_file.read()
            conversion_content = conversion_content.replace('@@PROJECT_NAME@@', project_name)
            conversion_content = conversion_content.replace('@@TYPE_PASCAL@@', pascal_str)
            conversion_content = conversion_content.replace('@@TYPE_SNAKE@@', snake_str)
            output_file.write(conversion_content)


def main():
    destination_dir = os.path.join(script_directory, "generated")
    delete_directory(destination_dir)
    os.makedirs(destination_dir)

    topic_directories = find_msg_directories(raisin_ws, ['src'])
    for topic_directory in topic_directories:
        create_interface(os.path.join(destination_dir, 'interfaces'), topic_directory)

    project_names = [os.path.basename(topic_directory) for topic_directory in topic_directories]

    ## helpers for finding interfaces
    helper_dir = os.path.join(destination_dir, 'helper')
    os.makedirs(helper_dir)
    generate_package_xml('raisin_bridge_helper', project_names, helper_dir)

    ## dummy cmakelist file
    cmakelists_content = "cmake_minimum_required(VERSION 3.22)"
    cmakelists_content += "\nfind_package(ament_cmake REQUIRED)"
    cmakelists_content += "\nproject(raisin_bridge_helper)\n"
    cmakelists_content += "\nament_package()"
    with open(os.path.join(helper_dir, 'CMakeLists.txt'), 'w') as output_file:
        output_file.write(cmakelists_content)

    cmake_content =  "\n".join(f"find_package({dep})" for dep in project_names)
    with open(os.path.join(helper_dir, 'find_interfaces.cmake'), 'w') as output_file:
        output_file.write(cmake_content)

if __name__ == '__main__':
    main()