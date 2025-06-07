import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose

def add_object(name, pose, size):
    scene = PlanningSceneInterface()
    rospy.sleep(2)  # Allow time for the scene to initialize

    # Define the object to be added
    object_pose = Pose()
    object_pose.position.x = pose[0]
    object_pose.position.y = pose[1]
    object_pose.position.z = pose[2]
    object_size = size  # Assuming size is a list [x, y, z]

    # Add the object to the planning scene
    scene.add_box(name, object_pose, size=object_size)
    rospy.loginfo(f"Added object: {name} at {pose} with size {size}")

def main():
    rospy.init_node('add_objects_node', anonymous=True)
    robot = RobotCommander()
    
    # Example usage
    add_object("box1", [0.5, 0.0, 0.5], [0.1, 0.1, 0.1])
    add_object("box2", [0.0, 0.5, 0.5], [0.1, 0.1, 0.1])

if __name__ == "__main__":
    main()