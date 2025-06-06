#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash || true
source /venv/bin/activate
exec "$@"
