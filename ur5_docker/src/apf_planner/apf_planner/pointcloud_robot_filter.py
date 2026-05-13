import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


class PointCloudFilter(Node):

    def __init__(self):
        super().__init__('pointcloud_filter')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/camera/points',   # corrected topic
            self.callback,
            10
        )

        self.publisher = self.create_publisher(
            PointCloud2,
            '/filtered_cloud',
            10
        )

        # approximate robot bounding box
        self.arm_min = np.array([-0.3, -0.3, 0.0])
        self.arm_max = np.array([0.3, 0.3, 1.2])

    def callback(self, msg):

        # extract xyz points
        points = np.array([
            [p[0], p[1], p[2]]
            for p in pc2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True
            )
        ])

        if len(points) == 0:
            return

        # detect points inside robot bounding box
        inside_box = np.all(
            (points >= self.arm_min) & (points <= self.arm_max),
            axis=1
        )

        # remove robot points
        filtered = points[~inside_box]

        # create new pointcloud
        cloud = pc2.create_cloud_xyz32(msg.header, filtered.tolist())

        self.publisher.publish(cloud)


def main():
    rclpy.init()
    node = PointCloudFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()