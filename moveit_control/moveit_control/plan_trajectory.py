import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
import time


class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.joint_names = [
            'remus_joint_a1', 'remus_joint_a2', 'remus_joint_a3', 'remus_joint_a4', 'remus_joint_a5', 'remus_joint_a6'
        ]  # Replace with your robot's joint names
        self.target_positions = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]  # Example target
        self.timer = self.create_timer(1.0, self.send_goal)

    def send_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available!')
            rclpy.shutdown()
            return

        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = 'robot1'  # Replace with your planning group

        # Set joint constraints
        constraints = Constraints()
        for name, pos in zip(self.joint_names, self.target_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)
        goal_msg.request = req

        self.get_logger().info('Sending goal to MoveGroup...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.timer.cancel()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Motion plan result received.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    rclpy.spin(node)

if __name__ == '__main__':
    main()