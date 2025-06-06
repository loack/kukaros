FROM ros:jazzy

# Set non-interactive frontend
ENV DEBIAN_FRONTEND=noninteractive

# Update & install ROS 2 development tools and GUI tools
RUN apt update && apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-moveit \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    git \
    build-essential \
    libeigen3-dev \
    libpython3-dev \
    && rm -rf /var/lib/apt/lists/*

# Set up working directory
WORKDIR /ros2_ws

# Create a virtual environment for Python packages
RUN apt update && apt install -y python3-venv && \
    python3 -m venv /venv && \
    /venv/bin/pip install --upgrade pip && \
    /venv/bin/pip install ikpy numpy matplotlib

# Source ROS setup and colcon environment in entrypoint
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
