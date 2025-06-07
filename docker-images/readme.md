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
docker run -it moveit/moveit2:main-jazzy-tutorial-source