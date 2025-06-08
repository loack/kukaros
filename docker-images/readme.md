Build container :

docker build -t ros2-dev .

Run container : 

xhost +local:docker
docker run -it --rm \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$HOME/ros2_ws:/ros2_ws" \
    --name ros2-dev \
    ros2-dev

ros2 launch moveit2_tutorials demo.launch.py

#setup assistant
ros2 launch moveit_setup_assistant setup_assistant.launch.py

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/kuka_experimental/kuka_kr210_support


run with : 

cd /kukabots/kukaros/docker-images/moveit_kr210
DOCKER_IMAGE=main-jazzy-tutorial-source docker-compose run gpu

new terminal : 
docker exec -it moveit_kr210_gpu_run_8a439b28588b bash

source /opt/ros/jazzy/setup.bash
source /root/ws_moveit/install/setup.bash

ros2 topic echo /joint_states

List All Nodes Publishing to /joint_states
ros2 topic info /joint_states
ros2 node list

hardware interfaces
ros2 control list_hardware_components
ros2 control list_hardware_interfaces