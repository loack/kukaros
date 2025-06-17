import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
import tkinter as tk
from tkinter import filedialog
import threading
import json
import time

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)

    def load_and_publish_trajectory(self):
        file_path = filedialog.askopenfilename(filetypes=[("JSON Files", "*.json")])
        if not file_path:
            return

        try:
            with open(file_path, 'r') as f:
                points = json.load(f)

            if not isinstance(points, list) or not all(len(p) == 3 for p in points):
                self.get_logger().error("Invalid trajectory format.")
                return

            marker = Marker()
            marker.header.frame_id = "world"  # or "base_link", depending on your setup
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "trajectory"
            marker.id = 0
            marker.type =Marker.SPHERE_LIST
            #Marker.LINE_STRIP 
            marker.action = Marker.ADD

            marker_size = 10/1000  # Line width in meters
            marker.scale.x = marker_size 
            marker.scale.y = marker_size
            marker.scale.z = marker_size

            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.6
            marker.color.a = 1.0
            marker.lifetime.sec = 0  # Persistent

            from geometry_msgs.msg import Point
            marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in points]
            self.get_logger().info(f"Publishing marker with frame_id: {marker.header.frame_id}")


            self.publisher_.publish(marker)
            self.get_logger().info("Trajectory published to RViz.")

        except Exception as e:
            self.get_logger().error(f"Failed to load trajectory: {e}")

def ros_spin(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = TrajectoryVisualizer()

    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    # Tkinter UI
    root = tk.Tk()
    root.title("Trajectory Loader")

    btn = tk.Button(root, text="Load Trajectory", command=node.load_and_publish_trajectory)
    btn.pack(padx=20, pady=20)

    def on_close():
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

if __name__ == '__main__':
    main()
