import os
import sys

# Get an environment variable (returns None if not found)
raisin_ws = os.getenv("RAISIN_WS")
sys.path.append(raisin_ws)
from raisin_workspace_setup import *


script_directory = os.path.dirname(os.path.realpath(__file__))
package_template = os.path.join(script_directory, 'src', 'templates', 'package.xml')
cmakelists_template = os.path.join(script_directory, 'src', 'templates', 'CMakeLists.txt')
conversion_hpp_template = os.path.join(script_directory, 'src', 'templates', 'conversion.hpp')
conversion_cpp_template = os.path.join(script_directory, 'src', 'templates', 'conversion.cpp')
script_directory = os.path.dirname(os.path.realpath(__file__))


def is_primitive_type(type_str: str) -> bool:
    """Check if the given type string is a primitive type."""
    primitive_types = [
        "bool", "byte", "char", "float32", "float64",
        "int8", "uint8", "int16", "uint16", "int32",
        "uint32", "int64", "uint64", "string", "wstring"
    ]
    return type_str in primitive_types

def is_vector_type(type_str: str) -> bool:
    """Check if the given type string represents a vector type."""
    return '[' in type_str or ']' in type_str

def get_base_type(type_str: str) -> str:
    """Extract the base type by removing any vector notation."""
    stripped_data_type = type_str.split('<', 1)[0]
    stripped_data_type = stripped_data_type.split('>', 1)[0]

    if not is_vector_type(stripped_data_type):
        return stripped_data_type  # No brackets, so it's just the base type
    return stripped_data_type.split('[')[0]  # Get the part before '['


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

    if (dependencies):
        cmakelists_content = cmakelists_content.replace('@@DEPENDENCIES@@',  "DEPENDENCIES  " + " ".join(dependencies))
    else:
        cmakelists_content = cmakelists_content.replace('@@DEPENDENCIES@@',  "")


    with open(os.path.join(destination_dir, 'CMakeLists.txt'), 'w') as output_file:
        output_file.write(cmakelists_content)


def conversion_str(msg_file):
    with open(msg_file, 'r') as msg_file_content:
        lines = msg_file_content.readlines()

    to_raisin = ""
    to_ros = ""
    for line in lines:
        line = line.strip()

        # Ignore comments by splitting at '#' and taking the part before it
        line = line.split('#', 1)[0].strip()

        # Skip empty lines
        if not line:
            continue

        parts = line.split()
        parts_in_two = line.split(' ', 1)

        if len(parts) < 4  and '=' not in parts_in_two[1]:
            initial_value = ''
            if len(parts) == 3:
                data_type, data_name, initial_value = parts
            else:
                data_type, data_name = parts

            vector_type = is_vector_type(data_type)
            base_type = get_base_type(data_type)
            primitive_type = is_primitive_type(base_type)
            
            data_name = re.sub(r'(?<!^)(?=[A-Z][a-z]|(?<=[a-z])[A-Z]|(?<=[0-9])(?=[A-Z]))', '_', data_name).lower()
            data_name = data_name.replace("__", "_")

            if primitive_type:
                to_raisin += f"\n  raisin_msg.{data_name} = ros_msg.{data_name};"
                to_ros += f"\n  ros_msg.{data_name} = raisin_msg.{data_name};"
            else:
                if vector_type:
                    to_raisin += f"\n  for (auto & item: ros_msg.{data_name})\
                        \n    raisin_msg.{data_name}.push_back(to_raisin_msg(item));"
                    to_ros += f"\n  for (auto & item: raisin_msg.{data_name})\
                        \n    ros_msg.{data_name}.push_back(to_ros_msg(item));"
                else:
                    to_raisin += f"\n  raisin_msg.{data_name} = to_raisin_msg(ros_msg.{data_name});"
                    to_ros += f"\n  ros_msg.{data_name} = to_ros_msg(raisin_msg.{data_name});"
                
    return to_raisin, to_ros

def find_dependencies(project_directory):
    project_name = os.path.basename(project_directory)

    msg_files = find_msg_files(project_directory)
    srv_files = find_srv_files(project_directory)

    dependencies = set([])

    for msg_file in msg_files:
        with open(msg_file, 'r') as msg_file_content:
            lines = msg_file_content.readlines()

        for line in lines:
            line = line.strip()

            # Ignore comments by splitting at '#' and taking the part before it
            line = line.split('#', 1)[0].strip()

            # Skip empty lines
            if not line:
                continue

            parts = line.split()
            parts_in_two = line.split(' ', 1)

            if len(parts) < 4  and '=' not in parts_in_two[1]:
                initial_value = ''
                if len(parts) == 3:
                    data_type, data_name, initial_value = parts
                else:
                    data_type, data_name = parts

                base_type = get_base_type(data_type)
                
                dependency = base_type.rpartition("/")[0]

                if dependency and dependency != project_name:
                    dependencies.add(dependency)

    for srv_file in srv_files:
        with open(srv_file, 'r') as srv_file_content:
            lines = srv_file_content.readlines()

        for line in lines:
            line = line.strip()

            # Ignore comments by splitting at '#' and taking the part before it
            line = line.split('#', 1)[0].strip()

            # Skip empty lines
            if not line or line == "---":
                continue

            parts = line.split()
            parts_in_two = line.split(' ', 1)

            if len(parts) < 4  and '=' not in parts_in_two[1]:
                initial_value = ''
                if len(parts) == 3:
                    data_type, data_name, initial_value = parts
                else:
                    data_type, data_name = parts

                base_type = get_base_type(data_type)
                
                dependency = base_type.rpartition("/")[0]
                
                if dependency and dependency != project_name:
                    dependencies.add(dependency)

    return dependencies



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

    dependencies = find_dependencies(project_directory)
    generate_package_xml(project_name, dependencies, destination_dir)
    generate_cmakelists_txt(project_name, dependencies, destination_dir)

    with open(os.path.join(destination_dir, "..", "..", 'interfaces.hpp'), 'a') as output_file:
        for msg_file in msg_files:
            snake_str = re.sub(r'(?<!^)(?=[A-Z][a-z]|(?<=[a-z])[A-Z]|(?<=[0-9])(?=[A-Z]))', '_', os.path.splitext(os.path.basename(msg_file))[0]).lower()
            snake_str = snake_str.replace("__", "_")
            output_file.write('#include <' + project_name + '/msg/' + snake_str + '.hpp>\n')
        for srv_file in srv_files:
            snake_str = re.sub(r'(?<!^)(?=[A-Z][a-z]|(?<=[a-z])[A-Z]|(?<=[0-9])(?=[A-Z]))', '_', os.path.splitext(os.path.basename(srv_file))[0]).lower()
            snake_str = snake_str.replace("__", "_")
            output_file.write('#include <' + project_name + '/srv/' + snake_str + '.hpp>\n')


    with open(os.path.join(destination_dir, 'conversion.cpp'), 'a') as output_file:
        output_file.write("#include <" + project_name + "/conversion.hpp>\n\n")
        for (dependency) in dependencies:
            output_file.write("#include <" + dependency + "/conversion.hpp>\n")
        for msg_file in msg_files:
            pascal_str = os.path.splitext(os.path.basename(msg_file))[0]
            snake_str = re.sub(r'(?<!^)(?=[A-Z][a-z]|(?<=[a-z])[A-Z]|(?<=[0-9])(?=[A-Z]))', '_', pascal_str).lower()
            snake_str = snake_str.replace("__", "_")
            with open(conversion_cpp_template, 'r') as template_file:
                conversion_content = template_file.read()
            conversion_content = conversion_content.replace('@@PROJECT_NAME@@', project_name)
            conversion_content = conversion_content.replace('@@TYPE_PASCAL@@', pascal_str)
            conversion_content = conversion_content.replace('@@TYPE_SNAKE@@', snake_str)
            to_raisin, to_ros = conversion_str(msg_file)
            conversion_content = conversion_content.replace('@@CONVERSION_TO_RAISIN@@', to_raisin)
            conversion_content = conversion_content.replace('@@CONVERSION_TO_ROS@@', to_ros)
            
            output_file.write(conversion_content)

    with open(os.path.join(destination_dir, 'conversion.hpp'), 'a') as output_file:
        for msg_file in msg_files:
            pascal_str = os.path.splitext(os.path.basename(msg_file))[0]
            snake_str = re.sub(r'(?<!^)(?=[A-Z][a-z]|(?<=[a-z])[A-Z]|(?<=[0-9])(?=[A-Z]))', '_', pascal_str).lower()
            snake_str = snake_str.replace("__", "_")
            with open(conversion_hpp_template, 'r') as template_file:
                conversion_content = template_file.read()
            conversion_content = conversion_content.replace('@@PROJECT_NAME@@', project_name)
            conversion_content = conversion_content.replace('@@TYPE_PASCAL@@', pascal_str)
            conversion_content = conversion_content.replace('@@TYPE_SNAKE@@', snake_str)
            
            output_file.write(conversion_content)


def main():
    destination_dir = os.path.join(script_directory, "generated")
    delete_directory(destination_dir)
    os.makedirs(destination_dir)

    topic_directories = find_msg_directories(raisin_ws, ['src', 'messages'])
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
    cmakelists_content += "\nset(CMAKE_PREFIX_PATH $ENV{RAISIN_WS}/install)\n"
    cmakelists_content += "\nfind_package(raisin_network REQUIRED)"
    for project in project_names:
        cmakelists_content += f"\nfind_package({project} REQUIRED)"
    cmakelists_content += "\nfind_package(ament_cmake REQUIRED)\n"
    cmakelists_content += "\nadd_library(raisin_bridge_helper"
    for project in project_names:
        cmakelists_content += f" ../interfaces/{project}/conversion.cpp"
    cmakelists_content += ")\nament_target_dependencies(raisin_bridge_helper"
    for project in project_names:
        cmakelists_content += f" {project}"
    cmakelists_content += ")\ntarget_link_libraries(raisin_bridge_helper raisin_network::raisin_network)"
    cmakelists_content += "\ntarget_include_directories(raisin_bridge_helper PRIVATE  $ENV{RAISIN_WS}/install)"
    cmakelists_content += "\ntarget_include_directories(raisin_bridge_helper PRIVATE  ../interfaces)"
    cmakelists_content += "\n\nament_package()"

    with open(os.path.join(helper_dir, 'CMakeLists.txt'), 'w') as output_file:
        output_file.write(cmakelists_content)


if __name__ == '__main__':
    main()