from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("remus", package_name="moveit_config").to_moveit_configs()
    demo_launch = generate_demo_launch(moveit_config)
    
    add_boxes_node = Node(
        package="moveit_control",              # Your package name
        executable="add_boxes",                # Entry point defined in setup.py
        output="screen"
    )

    trajectory_planner_node = Node(
        package="moveit_control",              # Your package name
        executable="plan_trajectory",           # Entry point defined in setup.py
        output="screen"
    )

    return LaunchDescription([
        *demo_launch.entities,
        add_boxes_node,
        trajectory_planner_node
    ])