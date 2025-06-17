import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import tkinter as tk


class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.publisher_ = self.create_publisher(String, 'trigger_action', 10)

    def button_callback(self):
        msg = String()
        msg.data = 'Button was pressed!'
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.publisher_.publish(msg)


def ros_spin(node):
    """Run ROS 2 spinning in a separate thread."""
    rclpy.spin(node)


def main():
    rclpy.init()
    node = GuiNode()

    # Start ROS 2 spinning in a separate thread
    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    # Build simple GUI with tkinter
    root = tk.Tk()
    root.title("RViz Action Trigger")

    btn = tk.Button(root, text="Trigger Action", command=node.button_callback)
    btn.pack(padx=20, pady=20)

    # Clean shutdown on close
    def on_close():
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


if __name__ == '__main__':
    main()
