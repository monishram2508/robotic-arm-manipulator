import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

from moveit2 import MoveIt2


class UR5PickPlace(Node):

    def __init__(self):

        super().__init__("ur5_pick_place_controller")

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
            group_name="ur_manipulator"
        )

    def move_to_pose(self, x, y, z):

        pose = Pose()

        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        pose.orientation.w = 1.0

        self.moveit.move_to_pose(
            position=[x, y, z],
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            cartesian=True
        )

    def pick_object(self, obj_pose):

        self.move_to_pose(obj_pose[0], obj_pose[1], obj_pose[2] + 0.1)

        self.get_logger().info("Open gripper")

        self.move_to_pose(obj_pose[0], obj_pose[1], obj_pose[2])

        self.get_logger().info("Close gripper")

        self.move_to_pose(obj_pose[0], obj_pose[1], obj_pose[2] + 0.15)

    def place_object(self, place_pose):

        self.move_to_pose(place_pose[0], place_pose[1], place_pose[2])

        self.get_logger().info("Open gripper")

        self.move_to_pose(place_pose[0], place_pose[1], place_pose[2] + 0.15)


def main():

    rclpy.init()

    node = UR5PickPlace()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()