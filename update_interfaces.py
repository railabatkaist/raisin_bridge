import os
import sys

# Get an environment variable (returns None if not found)
raisin_ws = os.getenv("RAISIN_WS")
sys.path.append(raisin_ws)
from raisin_workspace_setup import *


script_directory = os.path.dirname(os.path.realpath(__file__))
package_template = os.path.join(script_directory, 'src', 'templates', 'package.xml')
cmakelists_template = os.path.join(script_directory, 'src', 'templates', 'CMakeLists.txt')
script_directory = os.path.dirname(os.path.realpath(__file__))

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


    with open(package_template, 'r') as template_file:
        package_content = template_file.read()
    with open(cmakelists_template, 'r') as template_file:
        cmakelists_content = template_file.read()

    package_content = package_content.replace('@@PROJECT_NAME@@', project_name)
    cmakelists_content = cmakelists_content.replace('@@PROJECT_NAME@@', project_name)

    with open(os.path.join(destination_dir, 'package.xml'), 'w') as output_file:
        output_file.write(package_content)
    with open(os.path.join(destination_dir, 'CMakeLists.txt'), 'w') as output_file:
        output_file.write(cmakelists_content)



def main():
    destination_dir = os.path.join(script_directory, "interfaces")
    delete_directory(destination_dir)
    os.makedirs(destination_dir)

    topic_directories = find_msg_directories(raisin_ws, ['src'])
    for topic_directory in topic_directories:
        create_interface(destination_dir, topic_directory)

if __name__ == '__main__':
    main()