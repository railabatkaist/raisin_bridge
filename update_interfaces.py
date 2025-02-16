import os
import sys

# Get an environment variable (returns None if not found)
raisin_ws = os.getenv("RAISIN_WS")
sys.path.append(raisin_ws)
from raisin_workspace_setup import *

def copy_msg_file(msg_file, destination_dir, project_directory):
    project_name = os.path.basename(project_directory)

    # Determine the target directory in interfaces/<project_name>/msg
    destination_file = os.path.join(destination_dir, project_name, 'msg')
    os.makedirs(destination_file, exist_ok=True)
    shutil.copy2(msg_file, destination_file)


def copy_srv_file(srv_file, destination_dir, project_directory):
    project_name = os.path.basename(project_directory)

    destination_file = os.path.join(destination_dir, project_name, 'srv')
    os.makedirs(destination_file, exist_ok=True)
    shutil.copy2(srv_file, destination_file)

def main():
    script_directory = os.path.dirname(os.path.realpath(__file__))
    destination_dir = os.path.join(script_directory, "interfaces")
    delete_directory(destination_dir)
    os.makedirs(destination_dir)

    topic_directories = find_msg_directories(raisin_ws, ['src', 'messages'])
    for topic_directory in topic_directories:
        project_name = os.path.basename(topic_directory)
        
        # Handle .msg files
        msg_files = find_msg_files(topic_directory)
        for msg_file in msg_files:
            copy_msg_file(msg_file, destination_dir, topic_directory)

        # Handle .srv files
        srv_files = find_srv_files(topic_directory)
        for srv_file in srv_files:
            copy_srv_file(srv_file, destination_dir, topic_directory)

if __name__ == '__main__':
    main()