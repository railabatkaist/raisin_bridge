import os
import sys
from enum import Enum

# Get an environment variable (returns None if not found)
raisin_ws = os.getenv("RAISIN_WS")
sys.path.append(raisin_ws)
from raisin_workspace_setup import *

script_directory = os.path.dirname(os.path.realpath(__file__))

class SizeType(Enum):
    SCALAR = 1
    VECTOR = 2
    ARRAY = 3

def is_primitive_type(type_str: str) -> bool:
    """Check if the given type string is a primitive type."""
    primitive_types = [
        "bool", "byte", "char", "float32", "float64",
        "int8", "uint8", "int16", "uint16", "int32",
        "uint32", "int64", "uint64", "string", "wstring"
    ]
    return type_str in primitive_types

def get_size_type(type_str: str) -> SizeType:
    """Determine the size type of the given type string."""
    if "[]" in type_str:
        return SizeType.VECTOR
    elif '[' in type_str or '<' in type_str or '>' in type_str:
        return SizeType.ARRAY
    else:
        return SizeType.SCALAR

def get_base_type(type_str: str) -> str:
    """Extract the base type by removing any vector notation."""
    stripped_data_type = type_str.split('<', 1)[0]
    stripped_data_type = stripped_data_type.split('>', 1)[0]

    return stripped_data_type.split('[')[0]  # Get the part before '['


def generate_package_xml(project_name, dependencies, destination_dir):
    package_template = os.path.join(script_directory, 'src', 'templates', 'interfaces', 'package.xml')
    with open(package_template, 'r') as template_file:
        package_content = template_file.read()
        
    package_content = package_content.replace('@@PROJECT_NAME@@', project_name)
    package_content = package_content.replace('@@DEPENDENCIES@@',  "\n  ".join(f"<depend>{dep}</depend>" for dep in dependencies))

    with open(os.path.join(destination_dir, 'package.xml'), 'w') as output_file:
        output_file.write(package_content)

def generate_cmakelists_txt(project_name, dependencies, destination_dir):
    cmakelists_template = os.path.join(script_directory, 'src', 'templates', 'interfaces', 'CMakeLists.txt')
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

            size_type = get_size_type(data_type)
            base_type = get_base_type(data_type)
            primitive_type = is_primitive_type(base_type)
            
            data_name = re.sub(r'(?<!^)(?=[A-Z][a-z]|(?<=[a-z])[A-Z]|(?<=[0-9])(?=[A-Z]))', '_', data_name).lower()
            data_name = data_name.replace("__", "_")

            if primitive_type:
                if size_type == SizeType.VECTOR:
                    to_raisin += f"\n  for (const auto & item: ros_msg.{data_name})\
                        \n    raisin_msg.{data_name}.push_back(item);"
                    to_ros += f"\n  for (const auto & item: raisin_msg.{data_name})\
                        \n    ros_msg.{data_name}.push_back(item);"
                elif size_type == SizeType.ARRAY:
                    to_raisin += f"\n  for (int i = 0; i < ros_msg.{data_name}.size(); i++)\
                        \n    raisin_msg.{data_name}[i] = ros_msg.{data_name}[i];"
                    to_ros += f"\n  for (int i = 0; i < raisin_msg.{data_name}.size(); i++)\
                        \n    ros_msg.{data_name}[i] = raisin_msg.{data_name}[i];"
                else:
                    to_raisin += f"\n  raisin_msg.{data_name} = ros_msg.{data_name};"
                    to_ros += f"\n  ros_msg.{data_name} = raisin_msg.{data_name};"
            else:
                if size_type == SizeType.VECTOR:
                    to_raisin += f"\n  for (const auto & item: ros_msg.{data_name})\
                        \n    raisin_msg.{data_name}.push_back(to_raisin_msg(item));"
                    to_ros += f"\n  for (const auto & item: raisin_msg.{data_name})\
                        \n    ros_msg.{data_name}.push_back(to_ros_msg(item));"
                elif size_type == SizeType.ARRAY:
                    to_raisin += f"\n  for (int i = 0; i < ros_msg.{data_name}.size(); i++)\
                        \n    raisin_msg.{data_name}[i] = to_raisin_msg(ros_msg.{data_name}[i]);"
                    to_ros += f"\n  for (int i = 0; i < raisin_msg.{data_name}.size(); i++)\
                        \n    ros_msg.{data_name}[i] = to_ros_msg(raisin_msg.{data_name}[i]);"
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
            conversion_cpp_template = os.path.join(script_directory, 'src', 'templates', 'interfaces', 'conversion.cpp')
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
            conversion_hpp_template = os.path.join(script_directory, 'src', 'templates', 'interfaces', 'conversion.hpp')
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

    # topic_directories = find_msg_directories(raisin_ws, ['messages'])
    topic_directories = find_msg_directories(raisin_ws, ['install/messages'])
    for topic_directory in topic_directories:
        create_interface(os.path.join(destination_dir, 'interfaces'), topic_directory)

    project_names = [os.path.basename(topic_directory) for topic_directory in topic_directories]

    ## helpers for finding interfaces
    helper_dir = os.path.join(destination_dir, 'helper')
    os.makedirs(helper_dir)
    generate_package_xml('raisin_bridge_helper', project_names, helper_dir)

    ## cmakelist file for helper

    cmakelists_template = os.path.join(script_directory, 'src', 'templates', 'helper', 'CMakeLists.txt')
    with open(cmakelists_template, 'r') as template_file:
        cmakelists_content = template_file.read()

    content = ""
    for project in project_names:
        content += f"\nfind_package({project} REQUIRED)"
    content += "\nfind_package(ament_cmake REQUIRED)\n"
    content += "\nadd_library(raisin_bridge_helper conversion.cpp"
    for project in project_names:
        content += f" ../interfaces/{project}/conversion.cpp"
    content += ")\nament_target_dependencies(raisin_bridge_helper"
    for project in project_names:
        content += f" {project}"
    content += ")\nament_export_dependencies("
    for project in project_names:
        content += f" {project}"
    content += ")"

    cmakelists_content = cmakelists_content.replace('@@CONTENT@@', content)

    with open(os.path.join(helper_dir, 'CMakeLists.txt'), 'w') as output_file:
        output_file.write(cmakelists_content)



    ## conversion.hpp

    with open(os.path.join(script_directory, 'src', 'templates', 'helper', 'conversion.hpp'), 'r') as template_file:
        conversion_hpp_content = template_file.read()
    os.makedirs(os.path.join(helper_dir, 'include'), exist_ok=True)
    conversion_hpp_content = conversion_hpp_content.replace('@@INCLUDE_DEPENDENCIES@@',  "\n".join(f"#include <{project}/conversion.hpp>" for project in project_names))
    with open(os.path.join(helper_dir, 'include', 'conversion.hpp'), 'w') as output_file:
        output_file.write(conversion_hpp_content)

    ## conversion.cpp
    with open(os.path.join(script_directory, 'src', 'templates', 'helper', 'conversion.cpp'), 'r') as template_file:
        conversion_cpp_content = template_file.read()

    ## create subscribers, mapping between typename and subscriber
    subscribers_content = ""
    publishers_content = ""
    ros2_to_raisin_content = ""
    raisin_to_ros2_content = ""
    for project_directory in topic_directories:
        project_name = os.path.basename(project_directory)
        msg_files = find_msg_files(project_directory)
        for msg_file in msg_files:
            pascal_str = os.path.splitext(os.path.basename(msg_file))[0]
            snake_str = re.sub(r'(?<!^)(?=[A-Z][a-z]|(?<=[a-z])[A-Z]|(?<=[0-9])(?=[A-Z]))', '_', pascal_str).lower()
            # ros2_to_raisin_content += f"\n    if (type_name == \"{project_name}/msg/{pascal_str}\")"
            # ros2_to_raisin_content += f"    \n        register_ros2_to_raisin<{project_name}::msg::{pascal_str}, raisin::{project_name}::msg::{pascal_str}>(topic_name);"
            # raisin_to_ros2_content += f"\n    if (type_name == \"{project_name}/msg/{pascal_str}\")"
            # raisin_to_ros2_content += f"    \n        register_raisin_to_ros2<{project_name}::msg::{pascal_str}, raisin::{project_name}::msg::{pascal_str}>(topic_name);"
    conversion_cpp_content = conversion_cpp_content.replace('@@ROS2_TO_RAISIN@@', ros2_to_raisin_content)
    conversion_cpp_content = conversion_cpp_content.replace('@@RAISIN_TO_ROS2@@', raisin_to_ros2_content)

    with open(os.path.join(helper_dir, 'conversion.cpp'), 'a') as output_file:
        output_file.write(conversion_cpp_content)

if __name__ == '__main__':
    main()