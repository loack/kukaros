#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

#import kukavarproxy from same folder
from .kukavarproxy import KukaVarProxyClient

from .kvputils import set_speed, start_program, read_robot_state, read_axis_position, move_enable, ptp_motion,convert_position_string
from .kvputils import enable_linear_motion, enable_ptp_motion, go_home, convert2KukaPositionString
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
        #wait for everything to be ready
        time.sleep(5.0) 
        #Last interaction timer
        self.last_interaction_time = time.time()

        #target
        self.target_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #Initialize connection with KVP
        self.robot = KukaVarProxyClient('192.168.1.5',7000)
        self.robot.connect()
        self.logger.info("Connected to Kuka robot via KVP.")
        enable_linear_motion(self.robot, False)
        enable_ptp_motion(self.robot, True)
        start_program(self.robot)
        self.logger.info("Started KVP program on the robot.")
        set_speed(self.robot, 5)

        move_enable(self.robot, True)
        self.logger.info("Motion enabled on the robot.")
        
        #send robot to home position
        go_home(self.robot)
        self.logger.info("Moving robot to home position.")
        #wait for the robot to reach home position

        self.axis_string = read_axis_position(self.robot)
        self.logger.info(f"Current axis positions from robot: {self.axis_string }")
        #convert pos dict
        self.axis_robot = convert_position_string(self.axis_string)
        #self.logger.info(f"Current axis robot{self.axis_robot}")

        #Current axis position: {E6AXIS: A1 1.489927, A2 -59.15171, A3 112.9202, A4 1.845787, A5 -53.75971, A6 -1.091834, E1 0.0, E2 0.0, E3 0.0, E4 0.0, E5 0.0, E6 0.0}

        self.joint_state_sub = self.create_subscription(
            JointState, "joint_states", self.joint_state_callback, 100)
        self.logger.info("Subscriber for /joint_states created.")


    def joint_state_callback(self, msg: JointState):
        """
        Callback function for the /joint_states subscriber.
        It stores the latest joint state message.
        """
        #check how long from last interaction
        current_time = time.time()
        #log how long since last interaction
        self.logger.info(f"Time since last interaction: {current_time - self.last_interaction_time:.2f} seconds")
        self.last_joint_state = list(msg.position)
        self.last_interaction_time = current_time
        # Update the target joint positions
        ptp_motion(self.robot,msg.position)


    
    def print_info(self):
        axis_str = ', '.join(f"{v:.2f}" for v in self.axis_robot.values()) if isinstance(self.axis_robot, dict) else str(self.axis_robot)
        target_str = ', '.join(f"{v:.2f}" for v in self.target_joint_positions)
        print(f"\rActual axis: [{axis_str}] | Target: [{target_str}]", end='', flush=True)




        



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
