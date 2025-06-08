import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive, Mesh
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import MeshTriangle
import numpy as np


#from moveit.core.planning_scene import PlanningScene
from moveit_configs_utils import MoveItConfigsBuilder
from geometry_msgs.msg import Pose



class BoxAdder(Node):
    def __init__(self):
        super().__init__('box_adder')
        self.cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /apply_planning_scene service...')
        self.add_boxes()
        #self.add_object()

    def add_object(self):
        self.get_logger().info('Ajout du maillage de collision avec moveit_py.')

        # STL path - make sure it's an absolute path
        mesh_file_path = '/root/ws_moveit/src/moveit_control/meshes/frame.stl'        
        # Obtenir un verrou sur la scène de planification pour la modifier
        with self.planning_scene_monitor.read_write() as scene:
            
            # Définir la pose de l'objet
            object_pose = Pose()
            object_pose.position.x = 0.5
            object_pose.position.y = 0.0
            object_pose.position.z = 0.5
            object_pose.orientation.w = 1.0

            load_ok = PlanningScene.load_geometry_from_file(mesh_file_path)
            if not load_ok:
                self.get_logger().error(f"Failed to load mesh from {mesh_file_path}")
                return
            self.get_logger().info("Objet 'table_moveit_py' ajouté à la scène.")

       

    def add_boxes(self):
        scene = PlanningScene()
        scene.is_diff = True

        # Add box
        box = CollisionObject()
        box.id = f'box_{1}'
        box.header.frame_id = 'world'
        box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.1, 1.0, 1.0])]
        pose = Pose()
        pose.position.x = 1.0
        pose.position.y = 0.5
        pose.position.z = 0.5
        box.primitive_poses = [pose]
        box.operation = CollisionObject.ADD
        scene.world.collision_objects.append(box)

        req = ApplyPlanningScene.Request()
        req.scene = scene
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Boxes and STL mesh added to planning scene.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = BoxAdder()
     # Arrêter le nœud après l'exécution
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()