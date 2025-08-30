import rospy
import numpy as np
from tf import transformations
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

from typing import Any


class Ros1Publisher:
    def __init__(self):
        rospy.init_node('mad_icp', anonymous=True, disable_signals=True)

        self.out_topic_odom = rospy.get_param('~out_topic_odom', '/odometry/imu')
        self.out_topic_cloud = rospy.get_param('~out_topic_cloud', '/cloud_output')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'imu_link')
        self.map_frame_id = rospy.get_param('~map_frame_id', 'map')

        # Setup subscriber and publisher
        self.odom_pub = rospy.Publisher(self.out_topic_odom, Odometry, queue_size=10)
        self.cloud_pub = rospy.Publisher(self.out_topic_cloud, PointCloud2, queue_size=10)

        rospy.loginfo("MAD-ICP ROS publisher started:")
        rospy.loginfo("  Output topic: %s", self.out_topic_odom)
        rospy.loginfo("  Output frame_id: %s", self.odom_frame_id)

    def __del__(self):
        rospy.loginfo("MAD-ICP ROS publisher shutting down")
        rospy.signal_shutdown("Shutting down MAD-ICP ROS publisher")
        rospy.sleep(1)

    def publish_imu(self, ts: np.float64, base_to_world: np.ndarray):

        assert base_to_world.shape == (4, 4), f"ncorrect base_to_world shape: {base_to_world.shape}"

        msg = Odometry()
        header = Header()
        header.stamp = rospy.Time(secs=int(ts // 1e9), nsecs=int(ts) % 1e9)
        msg.header = header
        msg.header.frame_id = self.map_frame_id
        msg.child_frame_id = self.odom_frame_id

        # TODO: apply NED to ENU conversion like in `imu_frame_remapper.py`

        quat = transformations.quaternion_from_matrix(base_to_world)
        position = base_to_world[:3, 3:]
        msg.pose.pose.orientation = Quaternion(*list(quat))
        msg.pose.pose.position = Point(*list(position))
        # `msg.pose.covariance` <- leaving as default (all zeros)
        # `msg.twist.twist.linear` <- looks like `linear_velocity` or `linear_acceleration`
        # `msg.twist.twist.angular` <- looks like `angular_velocity`
        # `msg.position = base_to_world[:3, 3:]` <- leaving as default (all zeros)
        # `msg.twist.covariance` seems to consist of covariances for `linear_acceleration` & `angular_velocity`

        # Publish the message
        self.odom_pub.publish(msg)
