#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Import the MoveItPy interface
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)

class KukaHWInterface(Node):
    """
    A simple ROS2 node that demonstrates reading robot joint states and
    sending a joint goal to a robot using the MoveItPy interface.
    """

    def __init__(self):
        """
        Initializes the node, subscribers, and the MoveItPy instance.
        """
        super().__init__("kuka_hw_controller")
        self.logger = get_logger("kuka_hw_controller")
        self.logger.info("--------------Initializing KukaHWInterface node----------------")

        # =========================================================================================
        # /// IMPORTANT: USER-CONFIGURABLE PARAMETERS ///
        # =========================================================================================
        self.planning_group = "robot1"
        self.joint_names = [
            "remus_joint_a1",
            "remus_joint_a2",
            "remus_joint_a3",
            "remus_joint_a4",
            "remus_joint_a5",
            "remus_joint_a6",
            "remus_joint_a7",
        ]
        self.target_joint_positions = [0.0, 0, 0.0, 0, 0.0, 0, 0]
        # =========================================================================================

        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 1)
        self.logger.info("Publisher for /joint_states created.")

        # Publish initial joint states
        self.publish_initial_joint_states()

        # 1. --- Subscriber for Reading Joint States ---
        # This subscriber listens to the /joint_states topic to get real-time
        # information about the robot's current joint positions.
        self.joint_state_sub = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, 100)
        self.logger.info("Subscriber for /joint_states created.")

        #publish home position joint_state every 10 seconds
        self.target_joint_positions = [0.0, 0.5, 0.0, 0, 0.0, 0, 0]
        #self.timer = self.create_timer(10.0, self.publish_initial_joint_states)
        #self.logger.info("Timer for publishing initial joint states created.")


    def joint_state_callback(self, msg: JointState):
        """
        Callback function for the /joint_states subscriber.
        It stores the latest joint state message.
        """
        # We simply store the latest message. For a more complex application,
        # you might process or filter this data.
        self.last_joint_state = msg
        # Uncomment the line below for verbose logging of joint states.
        #self.get_logger().info(f"Received joint states: {list(msg.position)}")


    def publish_initial_joint_states(self):
        """
        Publishes the initial joint states at node startup.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.target_joint_positions
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        self.joint_state_pub.publish(msg)
        self.logger.info("Initial joint states published.")

    def destroy_node(self):
        self.logger.info("Shutting down the simple controller node.")
        super().destroy_node()

def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args)

    try:
        hw_interface_node = KukaHWInterface()
        # Use a MultiThreadedExecutor to allow MoveItPy's background tasks
        # and the node's callbacks to run concurrently.
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        executor.add_node(hw_interface_node)
        executor.spin()

    except Exception as e:
        get_logger("main").error(f"An error occurred in the main execution loop: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
