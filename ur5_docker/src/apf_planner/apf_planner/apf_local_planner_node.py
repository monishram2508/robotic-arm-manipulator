import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from moveit2 import MoveIt2


class APFPlanner(Node):

    def __init__(self):

        super().__init__('apf_planner')

        # MoveIt2 interface
        self.moveit = MoveIt2(
            node=self,
            joint_names=[
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint"
            ],
            base_link_name="base_link_inertia",
            end_effector_name="tool0",
            group_name="ur_manipulator",
        )

        # APF parameters
        self.k_att = 1.0
        self.k_rep = 0.8
        self.repulsive_radius = 0.3
        self.step_size = 0.02

        self.goal = None
        self.obstacles = np.empty((0, 3))

        # Subscribers
        self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.goal_callback,
            10
        )

        self.create_subscription(
            PointCloud2,
            '/filtered_cloud',
            self.cloud_callback,
            10
        )

        # Control loop
        self.create_timer(0.1, self.control_loop)

    # Goal pose callback
    def goal_callback(self, msg):

        self.goal = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

    # Point cloud callback
    def cloud_callback(self, msg):

        points = np.array([
            [p[0], p[1], p[2]]
            for p in pc2.read_points(
                msg,
                field_names=("x", "y", "z"),
                skip_nans=True
            )
        ])

        self.obstacles = points

    # Attractive force
    def attractive_force(self, current):

        return self.k_att * (self.goal - current)

    # Repulsive force
    def repulsive_force(self, current):

        force = np.zeros(3)

        if len(self.obstacles) == 0:
            return force

        for obs in self.obstacles:

            diff = current - obs
            dist = np.linalg.norm(diff)

            if dist < 1e-6:
                continue

            if dist < self.repulsive_radius:

                rep = self.k_rep * (1.0/dist - 1.0/self.repulsive_radius) \
                      * (1.0/(dist**3)) * diff

                force += rep

        return force

    def compute_force(self, current):

        return self.attractive_force(current) + self.repulsive_force(current)

    # Control loop
    def control_loop(self):

        if self.goal is None:
            return

        pose = self.moveit.get_current_pose()

        current = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z
        ])

        force = self.compute_force(current)

        norm = np.linalg.norm(force)

        if norm < 1e-3:
            self.get_logger().info("Goal reached")
            return

        direction = force / norm
        next_position = current + self.step_size * direction

        self.moveit.move_to_pose(
            position=next_position.tolist(),
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            cartesian=True
        )


def main():

    rclpy.init()

    node = APFPlanner()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()