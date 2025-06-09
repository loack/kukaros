import os

from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
def generate_demo_launch(moveit_config, launch_package_path=None):
     """
     Launches a self contained demo
  
     launch_package_path is optional to use different launch and config packages
  
     Includes
      * static_virtual_joint_tfs
      * robot_state_publisher
      * move_group
      * moveit_rviz
      * warehouse_db (optional)
      * ros2_control_node + controller spawners
     """
     if launch_package_path == None:
         launch_package_path = moveit_config.package_path
  
     ld = LaunchDescription()
     ld.add_action(
         DeclareBooleanLaunchArg(
             "db",
             default_value=False,
             description="By default, we do not start a database (it can be large)",
         )
     )
     ld.add_action(
         DeclareBooleanLaunchArg(
             "debug",
             default_value=False,
             description="By default, we are not in debug mode",
         )
     )
     ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

     
     # If there are virtual joints, broadcast static tf by including virtual_joints launch
     virtual_joints_launch = (
         launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
     )
  
     if virtual_joints_launch.exists():
         ld.add_action(
             IncludeLaunchDescription(
                 PythonLaunchDescriptionSource(str(virtual_joints_launch)),
             )
         )
  
     # Given the published joint states, publish tf for the robot links
     ld.add_action(
         IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 str(launch_package_path / "launch/rsp.launch.py")
             ),
         )
     )
  
     ld.add_action(
         IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 str(launch_package_path / "launch/move_group.launch.py")
             ),
         )
     )
  
     # Run Rviz and load the default config to see the state of the move_group node
     ld.add_action(
         IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 str(launch_package_path / "launch/moveit_rviz.launch.py")
             ),
             condition=IfCondition(LaunchConfiguration("use_rviz")),
         )
     )
  
     # If database loading was enabled, start mongodb as well
     ld.add_action(
         IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 str(launch_package_path / "launch/warehouse_db.launch.py")
             ),
             condition=IfCondition(LaunchConfiguration("db")),
         )
     )
  
     # Fake joint driver
     ld.add_action(
         Node(
             package="controller_manager",
             executable="ros2_control_node",
             parameters=[
                 moveit_config.robot_description,
                 str(moveit_config.package_path / "config/ros2_controllers.yaml"),
             ],
         )
     )
  
     ld.add_action(
         IncludeLaunchDescription(
             PythonLaunchDescriptionSource(
                 str(launch_package_path / "launch/spawn_controllers.launch.py")
             ),
         )
     )
  
     return ld

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

    # Add the Python hardware interface node
    kuka_hw_interface_node = Node(
        package="kuka_hw_controller",
        executable="kuka_hw_interface",  # Make sure this matches your entry point in setup.py
        output="screen",
        parameters=[{
            "python_class": "kuka_hw_controller.KukaHWInterface"
        }]
    )

    return LaunchDescription([
        *demo_launch.entities,
        add_boxes_node,
        trajectory_planner_node,
        kuka_hw_interface_node
    ])

