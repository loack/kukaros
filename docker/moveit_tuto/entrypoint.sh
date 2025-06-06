#!/bin/bash
set -e

cd /root/ws_moveit
colcon build --packages-select kr210_urdf
source /root/ws_moveit/install/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py

exec /bin/bash

#chmod +x /home/lolo/kukabots/kukaros/docker/moveit_tuto/entrypoint.sh