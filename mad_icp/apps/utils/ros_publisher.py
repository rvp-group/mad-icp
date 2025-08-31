import rospy
import numpy as np
from tf import transformations
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
# from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler, quaternion_multiply

from mad_icp.src.pybind.pypeline import VectorEigen3d


quat_enu_from_ned = quaternion_from_euler(np.pi, 0, 0)  # Rx_180deg


def transform_quat_enu_from_ned(quat_orig: Quaternion) -> Quaternion:
    """
    Args:
        quat_orig: rotation from body coord frame to ground (NED)
    Output:
        quat_out: rotation from body coord frame to ground (ENU)
    """
    quat_ned_orig = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
    quat_out = quaternion_multiply(quat_enu_from_ned, quat_ned_orig)
    # rpy = euler_from_quaternion(quat_ned_orig)
    # print(f"Q NED: {quat_ned_orig} // RPY: {rpy} // Q ENU: {quat_out}")
    return Quaternion(*list(quat_out))


class Ros1Publisher:
    def __init__(self):
        rospy.init_node('mad_icp', anonymous=True, disable_signals=True)

        self.out_topic_odom = rospy.get_param('~out_topic_odom', '/odometry/imu')
        self.out_topic_cloud = rospy.get_param('~out_topic_cloud', '/cloud/complete')
        self.out_topic_cloud_current = rospy.get_param('~out_topic_cloud_current', '/cloud/current')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'imu_link')
        self.map_frame_id = rospy.get_param('~map_frame_id', 'map')

        # Setup subscriber and publisher
        self.odom_pub = rospy.Publisher(self.out_topic_odom, Odometry, queue_size=10)
        self.cloud_complete_pub = rospy.Publisher(self.out_topic_cloud, PointCloud2, queue_size=10)
        self.cloud_current_pub = rospy.Publisher(self.out_topic_cloud_current, PointCloud2, queue_size=10)

        rospy.loginfo("MAD-ICP ROS publisher started:")
        rospy.loginfo("  Output topic: %s", self.out_topic_odom)
        rospy.loginfo("  Output frame_id: %s", self.odom_frame_id)

        self.point_cloud_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]

    def __del__(self):
        rospy.loginfo("MAD-ICP ROS publisher shutting down")
        rospy.signal_shutdown("Shutting down MAD-ICP ROS publisher")
        rospy.sleep(1)

    def _prepare_cloud_msg(self, ts: np.float64, points: VectorEigen3d) -> PointCloud2:
        msg = PointCloud2()
        header = Header()
        header.stamp = rospy.Time(secs=int(ts // 1e9), nsecs=int(ts) % 1e9)
        msg.header = header
        msg.header.frame_id = self.map_frame_id

        # Convert VectorEigen3d to numpy array
        points = np.asarray(points)

        # Set basic point cloud parameters
        msg.height = 1  # unordered point cloud
        msg.width = len(points)  # number of points

        msg.fields = self.point_cloud_fields

        # Set other required parameters
        msg.is_bigendian = False
        msg.point_step = 12  # 3 * float32 (4 bytes each)
        msg.row_step = msg.point_step * msg.width

        # Convert points to bytes
        msg.data = points.astype(np.float32).tobytes()
        msg.is_dense = True  # no invalid points

        return msg

    def publish_current_cloud(
            self,
            ts: np.float64,
            current_leaves: VectorEigen3d,
        ):
        msg = self._prepare_cloud_msg(ts, current_leaves)
        self.cloud_current_pub.publish(msg)

    def publish_complete_cloud(
            self,
            ts: np.float64,
            model_leaves: VectorEigen3d,
        ):
        msg = self._prepare_cloud_msg(ts, model_leaves)
        self.cloud_pub.publish(msg)

    def publish_imu(self, ts: np.float64, base_to_world: np.ndarray):

        assert base_to_world.shape == (4, 4), f"ncorrect base_to_world shape: {base_to_world.shape}"

        msg = Odometry()
        header = Header()
        header.stamp = rospy.Time(secs=int(ts // 1e9), nsecs=int(ts) % 1e9)
        msg.header = header
        msg.header.frame_id = self.map_frame_id
        msg.child_frame_id = self.odom_frame_id

        # NOTE: quaternion already follows ENU convention, but original has odometry to map frame transformation
        quat = transformations.quaternion_from_matrix(base_to_world)  # odom_to_map
        quat[3] = -quat[3]  # invert the quaternion (map_to_odom)
        msg.pose.pose.orientation = Quaternion(*list(quat))

        position = base_to_world[:3, 3:]
        # body NED to body ENU frame conversion (Rx_180deg)
        position[1] = -position[1]
        position[2] = -position[2]
        msg.pose.pose.position = Point(*list(position))
        # `msg.pose.covariance` <- leaving as default (all zeros)
        # `msg.twist.twist.linear` <- looks like `linear_velocity` or `linear_acceleration`
        # `msg.twist.twist.angular` <- looks like `angular_velocity`
        # `msg.position = base_to_world[:3, 3:]` <- leaving as default (all zeros)
        # `msg.twist.covariance` seems to consist of covariances for `linear_acceleration` & `angular_velocity`

        # Publish the message
        self.odom_pub.publish(msg)
