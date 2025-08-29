import rospy
import numpy as np
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Header

from typing import Any


class ImuAndCloudPublisher:
    def __init__(self):
        rospy.init_node('mad_icp', anonymous=True)

        self.out_topic_imu = rospy.get_param('~out_topic_imu', '/odometry/imu')
        self.out_topic_cloud = rospy.get_param('~out_topic_cloud', '/cloud_output')
        self.new_frame_id = rospy.get_param('~new_frame_id', 'imu_link')

        # Setup subscriber and publisher
        self.imu_pub = rospy.Publisher(self.out_topic_imu, Imu, queue_size=10)
        self.cloud_pub = rospy.Publisher(self.out_topic_cloud, PointCloud2, queue_size=10)

        rospy.loginfo("MAD-ICP ROS publisher started:")
        rospy.loginfo("  Output topic: %s", self.out_topic_imu)
        rospy.loginfo("  Output frame_id: %s", self.new_frame_id)

    def publish_imu(self, ts: np.float64, base_to_world: np.ndarray):

        # TODO: assert transform.shape == (4, 4), f"ncorrect transform shape: {transform.shape}""

        # TODO: remove the below `msg_tmp` definition and fill in the `msg`` with `transform`
        msg_tmp = Imu()
        quat_out = [1, 0, 0, 0]

        msg = Imu()
        header = Header()
        header.stamp.sec = int(ts // 1e-9)
        header.stamp.nsec = int(ts) % 1e-9
        msg.header = header

        msg.orientation = Quaternion(*list(quat_out))
        # `msg.orientation_covariance` <- leaving as default (all zeros)
        msg.angular_velocity = msg_tmp.angular_velocity
        # `msg.angular_velocity_covariance` <- leaving as default (all zeros)
        msg.linear_acceleration = msg_tmp.linear_acceleration
        # `msg.linear_acceleration_covariance` <- leaving as default (all zeros)

        # Update the frame_id
        msg.header.frame_id = self.new_frame_id

        # Republish the message
        self.imu_pub.publish(msg)
