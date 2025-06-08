#!/bin/bash
set -e

cd /root/ws_moveit
colcon build --packages-select kr210_urdf moveit_config moveit_control
#pip3 intall trimesh
source /root/ws_moveit/install/setup.bash
ros2 launch moveit_config control.launch.py
#ros2 launch moveit_config move_group.launch.py
#ros2 launch moveit2_tutorials motion_planning_python_api_tutorial.launch.py

exec /bin/bash

#chmod +x /home/lolo/kukabots/kukaros/docker/moveit_tuto/entrypoint.sh

#cd /root/ws_moveit/install/kr210_urdf/share/kr210_urdf/meshes/collision/link_5.stl