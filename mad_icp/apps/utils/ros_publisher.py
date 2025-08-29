import rospy
import numpy as np
from tf import transformations
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion

from typing import Any


class Ros1Publisher:
    def __init__(self):
        rospy.init_node('mad_icp', anonymous=True)

        self.out_topic_odom = rospy.get_param('~out_topic_odom', '/odometry/imu')
        self.out_topic_cloud = rospy.get_param('~out_topic_cloud', '/cloud_output')
        self.new_frame_id = rospy.get_param('~new_frame_id', 'imu_link')

        # Setup subscriber and publisher
        self.odom_pub = rospy.Publisher(self.out_topic_odom, Odometry, queue_size=10)
        self.cloud_pub = rospy.Publisher(self.out_topic_cloud, PointCloud2, queue_size=10)

        rospy.loginfo("MAD-ICP ROS publisher started:")
        rospy.loginfo("  Output topic: %s", self.out_topic_odom)
        rospy.loginfo("  Output frame_id: %s", self.new_frame_id)

    def publish_imu(self, ts: np.float64, base_to_world: np.ndarray):

        assert base_to_world.shape == (4, 4), f"ncorrect base_to_world shape: {base_to_world.shape}"

        msg = Odometry()
        header = Header()
        header.stamp = rospy.Time(secs=int(ts // 1e9), nsecs=int(ts) % 1e9)
        msg.header = header
        msg.header.frame_id = self.new_frame_id

        quat = transformations.quaternion_from_matrix(base_to_world)
        quat_out = [1, 0, 0, 0]

        position = base_to_world[3, :3]

        msg.orientation = Quaternion(*list(quat_out))
        # `msg.orientation_covariance` <- leaving as default (all zeros)
        # `msg.angular_velocity` <- leaving as default (all zeros)
        # `msg.angular_velocity_covariance` <- leaving as default (all zeros)
        # `msg.linear_acceleration` <- leaving as default (all zeros)
        # `msg.linear_acceleration_covariance` <- leaving as default (all zeros)



        # Republish the message
        self.odom_pub.publish(msg)
