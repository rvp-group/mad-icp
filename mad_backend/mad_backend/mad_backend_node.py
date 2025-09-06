import rclpy
from rclpy.node import Node
from mad_icp_ros_interfaces.msg import Frame   # <-- import the custom msg
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

from pyscancontext import *

class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_listener')
        # Adjust topic name and QoS to match the publisher

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.sub = self.create_subscription(
            Frame,               # message type from the other package
            '/keyframes',        # topic name
            self.on_msg,         # callback
            qos                  # QoS depth (or use QoSProfile if needed)
        )
        self.get_logger().info('FrameListener ready, waiting for frames...')

        scm = SCManager()

    def on_msg(self, msg: Frame):
        # Access fields from the custom message
        self.get_logger().info(
            f"Got Frame: id='{msg.seq}', stamp={msg.stamp.sec}.{msg.stamp.nanosec}"
        )

        cloud_mat = np.array(msg.cloud).reshape(3, msg.cloud_size)

        print("Cloud shape:", cloud_mat.shape)
        scm.add_node(scd)
        nn_idx, nn_dist, yaw_diff = scm.detect_loop()

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()