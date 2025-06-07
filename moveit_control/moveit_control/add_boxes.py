import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class BoxAdder(Node):
    def __init__(self):
        super().__init__('box_adder')
        self.cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /apply_planning_scene service...')
        self.add_boxes()

    def add_boxes(self):
        scene = PlanningScene()
        scene.is_diff = True

        for i in range(3):
            box = CollisionObject()
            box.id = f'box_{i}'
            box.header.frame_id = 'world'
            box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.1, 0.1, 0.1])]
            pose = Pose()
            pose.position.x = 0.4 + i * 0.2
            pose.position.y = 0.0
            pose.position.z = 0.1
            box.primitive_poses = [pose]
            box.operation = CollisionObject.ADD
            scene.world.collision_objects.append(box)

        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Boxes added to planning scene.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = BoxAdder()

if __name__ == '__main__':
    main()