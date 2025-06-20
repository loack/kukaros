import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject,ObjectColor
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

    def make_box(self, name, dimensions,position,colorRGB=[1.0, 0.0, 0.0]):
        box = CollisionObject()
        box.id = name
        box.header.frame_id = 'world'
        box.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=dimensions)]
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        box.primitive_poses = [pose]
        box.operation = CollisionObject.ADD

        # Define color for the box
        color = ObjectColor()
        color.id = box.id
        color.color.r = colorRGB[0]
        color.color.g = colorRGB[1]
        color.color.b = colorRGB[2]
        color.color.a = 1.0  # Opaque
        
        return box, color
    
    def add_box_to_scene(self, scene, name, dimensions, position, colorRGB=[1.0, 0.0, 0.0]):
        box, color = self.make_box(name, dimensions, position, colorRGB)
        scene.is_diff = True  # Indicate that this is a diff update
        scene.world.collision_objects.append(box)
        scene.object_colors.append(color)
        return scene

    def add_boxes(self):
        scene = PlanningScene()
        scene.is_diff = True
         # Define the frame_id for the scene                           x    y    z
        scene = self.add_box_to_scene(scene, 'sol', [6.0, 6.0, 0.1], [0.0, 0.0, -0.05], [1.0, 1.0, 1.0])
        scene = self.add_box_to_scene(scene, 'palette', [1.2, 1.0, 1.5], [0.0, -2.0, 0.75], [0.0, 1.0, 0.0])
        scene = self.add_box_to_scene(scene, 'voiture', [2.0, 4.8, 2.0], [4.0, -2.0, 1.0], [0.0, 0.0, 1.0])

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

if __name__ == '__main__':
    main()