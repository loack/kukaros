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
