#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

#import kukavarproxy from same folder
from .kukavarproxy import KukaVarProxyClient

from .kvputils import set_speed, start_program, read_robot_state, read_xyz_position, move_enable, ptp_motion

import time

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

        #Last interaction timer
        self.last_interaction_time = self.get_clock().now()

        #target
        self.target_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #Initialize connection with KVP
        '''self.robot = KukaVarProxyClient('192.168.1.5',7000)
        self.robot.connect()
        self.logger.info("Connected to Kuka robot via KVP.")
        start_program(self.robot)
        self.logger.info("Started KVP program on the robot.")
        set_speed(self.robot, 10)'''

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
        
        # =========================================================================================

        # Publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 1)
        self.logger.info("Publisher for /joint_states created.")

        self.joint_state_sub = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, 100)
        self.logger.info("Subscriber for /joint_states created.")


    def joint_state_callback(self, msg: JointState):
        """
        Callback function for the /joint_states subscriber.
        It stores the latest joint state message.
        """
        #check how long from last interaction
        current_time = self.get_clock().now()
        if (current_time - self.last_interaction_time).nanoseconds/1000000 > 100:  # 100 milisecond
            self.last_joint_state = msg
            self.get_logger().info(f"Received joint states: {list(msg.position)}")
            self.last_interaction_time = current_time
            #axis_states_robot = read_robot_state(self.robot)
            #self.get_logger().info(f"Current axis states from robot: {axis_states_robot}")
            #ptp_motion(self.robot,msg.position)




        



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
