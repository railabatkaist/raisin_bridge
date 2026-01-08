import os
import sys
from enum import Enum
import shutil

# Get an environment variable (returns None if not found)
raisin_ws = os.getenv("RAISIN_WS")
sys.path.append(raisin_ws)
from commands.setup import *
import commands.globals as g 
g.script_directory = raisin_ws

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


def msg_conversion(msg_file):
    interface_name = os.path.basename(os.path.dirname((os.path.dirname(msg_file))))
    pascal_str = os.path.splitext(os.path.basename(msg_file))[0]
    snake_str = re.sub(r'(?<!^)(?=[A-Z][a-z]|(?<=[a-z])[A-Z]|(?<=[0-9])(?=[A-Z]))', '_', pascal_str).lower()
    snake_str = snake_str.replace("__", "_")
    conversion_cpp_template = os.path.join(script_directory, 'templates', 'conversions', 'conversion_msg.cpp.in')
    with open(conversion_cpp_template, 'r') as template_file:
        conversion_content = template_file.read()
    conversion_content = conversion_content.replace('@@INTERFACE_NAME@@', interface_name)
    conversion_content = conversion_content.replace('@@TYPE_PASCAL@@', pascal_str)
    conversion_content = conversion_content.replace('@@TYPE_SNAKE@@', snake_str)
    
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
                        \n    raisin_msg.{data_name}.push_back(to_raisin(item));"
                    to_ros += f"\n  for (const auto & item: raisin_msg.{data_name})\
                        \n    ros_msg.{data_name}.push_back(to_ros(item));"
                elif size_type == SizeType.ARRAY:
                    to_raisin += f"\n  for (int i = 0; i < ros_msg.{data_name}.size(); i++)\
                        \n    raisin_msg.{data_name}[i] = to_raisin(ros_msg.{data_name}[i]);"
                    to_ros += f"\n  for (int i = 0; i < raisin_msg.{data_name}.size(); i++)\
                        \n    ros_msg.{data_name}[i] = to_ros(raisin_msg.{data_name}[i]);"
                else:
                    to_raisin += f"\n  raisin_msg.{data_name} = to_raisin(ros_msg.{data_name});"
                    to_ros += f"\n  ros_msg.{data_name} = to_ros(raisin_msg.{data_name});"
                

    conversion_content = conversion_content.replace('@@CONVERSION_TO_RAISIN@@', to_raisin)
    conversion_content = conversion_content.replace('@@CONVERSION_TO_ROS@@', to_ros)

    return conversion_content

def srv_conversion(srv_file):
    interface_name = os.path.basename(os.path.dirname((os.path.dirname(srv_file))))
    pascal_str = os.path.splitext(os.path.basename(srv_file))[0]
    snake_str = re.sub(r'(?<!^)(?=[A-Z][a-z]|(?<=[a-z])[A-Z]|(?<=[0-9])(?=[A-Z]))', '_', pascal_str).lower()
    snake_str = snake_str.replace("__", "_")
    conversion_cpp_template = os.path.join(script_directory, 'templates', 'conversions', 'conversion_srv.cpp.in')
    with open(conversion_cpp_template, 'r') as template_file:
        conversion_content = template_file.read()
    conversion_content = conversion_content.replace('@@INTERFACE_NAME@@', interface_name)
    conversion_content = conversion_content.replace('@@TYPE_PASCAL@@', pascal_str)
    conversion_content = conversion_content.replace('@@TYPE_SNAKE@@', snake_str)

    with open(srv_file, 'r') as srv_file_content:
        lines = srv_file_content.readlines()

    to_raisin = ["", ""]
    to_ros = ["", ""]
    index = 0 ## 0 for request, 1 for response

    for line in lines:
        line = line.strip()

        # Ignore comments by splitting at '#' and taking the part before it
        line = line.split('#', 1)[0].strip()

        # Skip empty lines
        if not line:
            continue
        if line == "---":
            index = 1
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
                    to_raisin[index] += f"\n  for (const auto & item: ros_msg.{data_name})\
                        \n    raisin_msg.{data_name}.push_back(item);"
                    to_ros[index] += f"\n  for (const auto & item: raisin_msg.{data_name})\
                        \n    ros_msg.{data_name}.push_back(item);"
                elif size_type == SizeType.ARRAY:
                    to_raisin[index] += f"\n  for (int i = 0; i < ros_msg.{data_name}.size(); i++)\
                        \n    raisin_msg.{data_name}[i] = ros_msg.{data_name}[i];"
                    to_ros[index] += f"\n  for (int i = 0; i < raisin_msg.{data_name}.size(); i++)\
                        \n    ros_msg.{data_name}[i] = raisin_msg.{data_name}[i];"
                else:
                    to_raisin[index] += f"\n  raisin_msg.{data_name} = ros_msg.{data_name};"
                    to_ros[index] += f"\n  ros_msg.{data_name} = raisin_msg.{data_name};"
            else:
                if size_type == SizeType.VECTOR:
                    to_raisin[index] += f"\n  for (const auto & item: ros_msg.{data_name})\
                        \n    raisin_msg.{data_name}.push_back(to_raisin(item));"
                    to_ros[index] += f"\n  for (const auto & item: raisin_msg.{data_name})\
                        \n    ros_msg.{data_name}.push_back(to_ros(item));"
                elif size_type == SizeType.ARRAY:
                    to_raisin[index] += f"\n  for (int i = 0; i < ros_msg.{data_name}.size(); i++)\
                        \n    raisin_msg.{data_name}[i] = to_raisin(ros_msg.{data_name}[i]);"
                    to_ros[index] += f"\n  for (int i = 0; i < raisin_msg.{data_name}.size(); i++)\
                        \n    ros_msg.{data_name}[i] = to_ros(raisin_msg.{data_name}[i]);"
                else:
                    to_raisin[index] += f"\n  raisin_msg.{data_name} = to_raisin(ros_msg.{data_name});"
                    to_ros[index] += f"\n  ros_msg.{data_name} = to_ros(raisin_msg.{data_name});"
                

    conversion_content = conversion_content.replace('@@CONVERSION_TO_RAISIN_REQ@@', to_raisin[0])
    conversion_content = conversion_content.replace('@@CONVERSION_TO_ROS_REQ@@', to_ros[0])

    conversion_content = conversion_content.replace('@@CONVERSION_TO_RAISIN_RES@@', to_raisin[1])
    conversion_content = conversion_content.replace('@@CONVERSION_TO_ROS_RES@@', to_ros[1])

    return conversion_content


def group_files_by_package(msg_files, srv_files):
    package_map = {} # { 'package_path': {'msgs': [], 'srvs': []} }

    for f in msg_files:
        pkg_dir = os.path.dirname(os.path.dirname(f))
        package_map.setdefault(pkg_dir, {'msgs': [], 'srvs': []})['msgs'].append(f)

    for f in srv_files:
        pkg_dir = os.path.dirname(os.path.dirname(f))
        package_map.setdefault(pkg_dir, {'msgs': [], 'srvs': []})['srvs'].append(f)

    return package_map


def find_dependencies(interface_name, msg_files, srv_files):
 
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

                if dependency and dependency != interface_name:
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
                
                if dependency and dependency != interface_name:
                    dependencies.add(dependency)

    return dependencies



def create_interface(destination_dir, project_directory, msg_files, srv_files):
    interface_name = os.path.basename(project_directory)
    destination_dir = os.path.join(destination_dir, 'interfaces', interface_name)

    dependencies = find_dependencies(interface_name, msg_files, srv_files)

    ## make directory and copy files
    msg_dir = os.path.join(destination_dir, 'msg')
    srv_dir = os.path.join(destination_dir, 'srv')
    os.makedirs(msg_dir, exist_ok=True)
    os.makedirs(srv_dir, exist_ok=True)

    for msg_file in msg_files:
        shutil.copy2(msg_file, msg_dir)
    for srv_file in srv_files:
        shutil.copy2(srv_file, srv_dir)

    ## packages.xml
    package_template = os.path.join(script_directory, 'templates', 'interfaces', 'package.xml.in')
    with open(package_template, 'r') as template_file:
        package_content = template_file.read()
        
    package_content = package_content.replace('@@INTERFACE_NAME@@', interface_name)
    package_content = package_content.replace('@@DEPENDENCIES@@',  "\n  ".join(f"<depend>{dep}</depend>" for dep in dependencies))

    with open(os.path.join(destination_dir, 'package.xml'), 'w') as output_file:
        output_file.write(package_content)

    ## cmakelists.xml
    cmakelists_template = os.path.join(script_directory, 'templates', 'interfaces', 'CMakeLists.txt.in')
    with open(cmakelists_template, 'r') as template_file:
        cmakelists_content = template_file.read()
        
    cmakelists_content = cmakelists_content.replace('@@INTERFACE_NAME@@', interface_name)
    cmakelists_content = cmakelists_content.replace('@@DEPENDENCIES@@',  " ".join(dependencies))

    with open(os.path.join(destination_dir, 'CMakeLists.txt'), 'w') as output_file:
        output_file.write(cmakelists_content)



def create_conversion(destination_dir, project_directory, msg_files, srv_files):
    interface_name = os.path.basename(project_directory)
    destination_dir = os.path.join(destination_dir, 'conversions', interface_name)

    dependencies = find_dependencies(interface_name, msg_files, srv_files)

    conversion_header_dir = os.path.join(destination_dir, 'include', interface_name)
    os.makedirs(conversion_header_dir, exist_ok=True)

    ## packages.xml
    package_template = os.path.join(script_directory, 'templates', 'conversions', 'package.xml.in')
    with open(package_template, 'r') as template_file:
        package_content = template_file.read()
        
    package_content = package_content.replace('@@INTERFACE_NAME@@', interface_name)
    package_content = package_content.replace('@@DEPENDENCIES@@',  "\n  ".join(f"<depend>{dep}_conversion</depend>" for dep in dependencies))

    with open(os.path.join(destination_dir, 'package.xml'), 'w') as output_file:
        output_file.write(package_content)

    ## cmakelists.xml
    cmakelists_template = os.path.join(script_directory, 'templates', 'conversions', 'CMakeLists.txt.in')
    with open(cmakelists_template, 'r') as template_file:
        cmakelists_content = template_file.read()
        
    cmakelists_content = cmakelists_content.replace('@@INTERFACE_NAME@@', interface_name)
    cmakelists_content = cmakelists_content.replace('@@DEPENDENCIES@@',  " ".join(f"{dep}_conversion" for dep in dependencies))

    with open(os.path.join(destination_dir, 'CMakeLists.txt'), 'w') as output_file:
        output_file.write(cmakelists_content)

    ## conversion.cpp
    with open(os.path.join(destination_dir, 'conversion.cpp'), 'w') as output_file:
        pass  # Opening in 'w' mode clears the file content
    with open(os.path.join(destination_dir, 'conversion.cpp'), 'a') as output_file:
        output_file.write("#include <" + interface_name + "/conversion.hpp>\n\n")
        with open(os.path.join(script_directory, 'templates', 'conversions', 'conversion_cpp_register'), 'r') as conversion_cpp_register:
            output_file.write(conversion_cpp_register.read())
        # Handle msg files
        for msg_file in msg_files:
            conversion_content = msg_conversion(msg_file)
            output_file.write(conversion_content)

        # Handle srv files
        for srv_file in srv_files:
            conversion_content = srv_conversion(srv_file)
            output_file.write(conversion_content)

        # register msg
        output_file.write("extern \"C\" {")
        output_file.write("\n  void register_ros2_to_raisin_msg(BridgeNode * bridgeNode, std::string type_name, std::string topic_name)\n  {\n")
        for msg_file in msg_files:
            pascal_str = os.path.splitext(os.path.basename(msg_file))[0]
            output_file.write(f"    if(type_name == \"{pascal_str}\")\n      register_ros2_to_raisin_msg<{interface_name}::msg::{pascal_str}, raisin::{interface_name}::msg::{pascal_str}>(bridgeNode, topic_name);\n")
        output_file.write("  }\n}\n")
        output_file.write("extern \"C\" {")
        output_file.write("\n  void register_raisin_to_ros2_msg(BridgeNode * bridgeNode, std::string type_name, std::string topic_name)\n  {\n")
        for msg_file in msg_files:
            pascal_str = os.path.splitext(os.path.basename(msg_file))[0]
            output_file.write(f"    if(type_name == \"{pascal_str}\")\n      register_raisin_to_ros2_msg<{interface_name}::msg::{pascal_str}, raisin::{interface_name}::msg::{pascal_str}>(bridgeNode, topic_name);\n")
        output_file.write("  }\n}\n")

        # register srv
        output_file.write("extern \"C\" {")
        output_file.write("\n  void register_ros2_to_raisin_srv(BridgeNode * bridgeNode, std::string type_name, std::string service_name)\n  {\n")
        for srv_file in srv_files:
            pascal_str = os.path.splitext(os.path.basename(srv_file))[0]
            output_file.write(f"    if(type_name == \"{pascal_str}\")\n      register_ros2_to_raisin_srv<{interface_name}::srv::{pascal_str}, raisin::{interface_name}::srv::{pascal_str}, {interface_name}::srv::{pascal_str}::Request, raisin::{interface_name}::srv::{pascal_str}::Request, {interface_name}::srv::{pascal_str}::Response, raisin::{interface_name}::srv::{pascal_str}::Response>(bridgeNode, service_name);\n")
        output_file.write("  }\n}\n")
        output_file.write("extern \"C\" {")
        output_file.write("\n  void register_raisin_to_ros2_srv(BridgeNode * bridgeNode, std::string type_name, std::string service_name)\n  {\n")
        for srv_file in srv_files:
            pascal_str = os.path.splitext(os.path.basename(srv_file))[0]
            output_file.write(f"    if(type_name == \"{pascal_str}\")\n      register_raisin_to_ros2_srv<{interface_name}::srv::{pascal_str}, raisin::{interface_name}::srv::{pascal_str}, {interface_name}::srv::{pascal_str}::Request, raisin::{interface_name}::srv::{pascal_str}::Request, {interface_name}::srv::{pascal_str}::Response, raisin::{interface_name}::srv::{pascal_str}::Response>(bridgeNode, service_name);\n")
        output_file.write("  }\n}")


    ## conversion.hpp
    with open(os.path.join(conversion_header_dir, 'conversion.hpp'), 'w') as output_file:
        pass  # Opening in 'w' mode clears the file content
    with open(os.path.join(conversion_header_dir, 'conversion.hpp'), 'a') as output_file:
        for (dependency) in dependencies:
            output_file.write("#include <" + dependency + "/conversion.hpp>\n")
        output_file.write('#include <raisin_bridge/raisin_bridge.hpp>\n\n')

        for msg_file in msg_files:
            pascal_str = os.path.splitext(os.path.basename(msg_file))[0]
            snake_str = re.sub(r'(?<!^)(?=[A-Z][a-z]|(?<=[a-z])[A-Z]|(?<=[0-9])(?=[A-Z]))', '_', pascal_str).lower()
            snake_str = snake_str.replace("__", "_")
            conversion_hpp_template = os.path.join(script_directory, 'templates', 'conversions', 'conversion_msg.hpp.in')
            with open(conversion_hpp_template, 'r') as template_file:
                conversion_content = template_file.read()
            conversion_content = conversion_content.replace('@@INTERFACE_NAME@@', interface_name)
            conversion_content = conversion_content.replace('@@TYPE_PASCAL@@', pascal_str)
            conversion_content = conversion_content.replace('@@TYPE_SNAKE@@', snake_str)
            
            output_file.write(conversion_content)

        for srv_file in srv_files:
            pascal_str = os.path.splitext(os.path.basename(srv_file))[0]
            snake_str = re.sub(r'(?<!^)(?=[A-Z][a-z]|(?<=[a-z])[A-Z]|(?<=[0-9])(?=[A-Z]))', '_', pascal_str).lower()
            snake_str = snake_str.replace("__", "_")
            conversion_hpp_template = os.path.join(script_directory, 'templates', 'conversions', 'conversion_srv.hpp.in')
            with open(conversion_hpp_template, 'r') as template_file:
                conversion_content = template_file.read()
            conversion_content = conversion_content.replace('@@INTERFACE_NAME@@', interface_name)
            conversion_content = conversion_content.replace('@@TYPE_PASCAL@@', pascal_str)
            conversion_content = conversion_content.replace('@@TYPE_SNAKE@@', snake_str)
            
            output_file.write(conversion_content)
            
def main():
    destination_dir = os.path.join(script_directory, "generated")
    os.makedirs(destination_dir, exist_ok=True)
    os.makedirs(os.path.join(destination_dir, 'interfaces'), exist_ok=True)
    os.makedirs(os.path.join(destination_dir, 'conversions'), exist_ok=True)

    # topic_directories = find_msg_directories(raisin_ws, ['messages'])
    # topic_directories = find_msg_directories(raisin_ws, ['install/messages/geometry_msgs', 'install/messages/std_msgs', 'install/messages/builtin_interfaces'])

    msg_list, srv_list = find_interface_files(
        search_directories=['install/messages'],
        interface_types=['msg', 'srv']
    )
    
    package_map = group_files_by_package(msg_list, srv_list)


    for pkg_dir, files in package_map.items():
        pkg_name = os.path.basename(pkg_dir)
        
        if pkg_name in ["raisin_thread_pool", "raisin_ffmpeg_image_transport"]:
            continue
            
        create_interface(destination_dir, pkg_dir, files['msgs'], files['srvs'])
        create_conversion(destination_dir, pkg_dir, files['msgs'], files['srvs'])

if __name__ == '__main__':
    main()