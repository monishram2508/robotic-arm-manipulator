#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, WorkspaceParameters,
                              Constraints, PositionConstraint, BoundingVolume)
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import time

class APFPickPlace(Node):
    def __init__(self):
        super().__init__("apf_pick_place")
        self.k_att = 1.0
        self.k_rep = 0.5
        self.d0 = 0.3
        self.step_size = 0.05
        self.pick_pos  = np.array([0.546,  0.494, 1.062])
        self.place_pos = np.array([0.718, -0.282, 1.137])
        self.obstacles = [np.array([0.0, 0.0, 0.9])]

        self._ac = ActionClient(self, MoveGroup, '/move_action')
        self._gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_position_controller/joint_trajectory', 10)

        self.get_logger().info("Waiting for MoveGroup...")
        self._ac.wait_for_server()
        self.get_logger().info("Connected!")
        time.sleep(1.0)
        self.run()

    def attractive(self, cur, goal):
        d = goal - cur
        n = np.linalg.norm(d)
        return self.k_att * d / n if n > 0.01 else np.zeros(3)

    def repulsive(self, cur):
        f = np.zeros(3)
        for obs in self.obstacles:
            d = cur - obs
            dist = np.linalg.norm(d)
            if 0 < dist < self.d0:
                mag = self.k_rep * (1/dist - 1/self.d0) / dist**2
                f += mag * d / dist
        return f

    def apf_path(self, start, goal, max_steps=60):
        path, cur = [start.copy()], start.copy()
        for _ in range(max_steps):
            if np.linalg.norm(cur - goal) < self.step_size:
                path.append(goal.copy()); break
            f = self.attractive(cur, goal) + self.repulsive(cur)
            n = np.linalg.norm(f)
            cur = cur + self.step_size * f / n if n > 0.001 else cur
            path.append(cur.copy())
        return path

    def move_to_pose(self, x, y, z, label=""):
        self.get_logger().info(f"→ {label}: ({x:.3f}, {y:.3f}, {z:.3f})")

        ps = PoseStamped()
        ps.header.frame_id = "world"
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.x = -0.707
        ps.pose.orientation.y = 0.0
        ps.pose.orientation.z = 0.0
        ps.pose.orientation.w = 0.707

        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.02]

        bv = BoundingVolume()
        bv.primitives = [sp]
        bv.primitive_poses = [ps.pose]

        pc = PositionConstraint()
        pc.header.frame_id = "world"
        pc.link_name = "tool0"
        pc.constraint_region = bv
        pc.weight = 1.0

        c = Constraints()
        c.position_constraints = [pc]

        ws = WorkspaceParameters()
        ws.header.frame_id = "world"
        ws.min_corner.x = -2.0; ws.min_corner.y = -2.0; ws.min_corner.z = -0.5
        ws.max_corner.x =  2.0; ws.max_corner.y =  2.0; ws.max_corner.z =  3.0

        req = MotionPlanRequest()
        req.group_name = "ur5_manipulator"
        req.num_planning_attempts = 20
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.workspace_parameters = ws
        req.goal_constraints = [c]

        goal = MoveGroup.Goal()
        goal.request = req

        future = self._ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn(f"Rejected: {label}")
            return False
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        ok = result_future.result().result.error_code.val == 1
        self.get_logger().info(f"{'✓' if ok else '✗'} {label} (code={result_future.result().result.error_code.val})")
        return ok

    def move_home(self):
        from moveit_msgs.msg import JointConstraint
        self.get_logger().info("→ HOME")
        jcs = []
        for name, pos in [("shoulder_pan_joint", 0.0),
                          ("shoulder_lift_joint", -1.57),
                          ("elbow_joint", 0.0),
                          ("wrist_1_joint", -1.57),
                          ("wrist_2_joint", 0.0),
                          ("wrist_3_joint", 0.0)]:
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            jcs.append(jc)
        c = Constraints(); c.joint_constraints = jcs
        req = MotionPlanRequest()
        req.group_name = "ur5_manipulator"
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.goal_constraints = [c]
        goal = MoveGroup.Goal(); goal.request = req
        future = self._ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if gh.accepted:
            rclpy.spin_until_future_complete(self, gh.get_result_async())
        time.sleep(1.0)

    def gripper(self, pos):
        msg = JointTrajectory()
        msg.joint_names = ['robotiq_85_left_knuckle_joint']
        pt = JointTrajectoryPoint()
        pt.positions = [pos]
        pt.time_from_start = Duration(sec=1)
        msg.points = [pt]
        self._gripper_pub.publish(msg)
        time.sleep(1.5)

    def run(self):
        self.get_logger().info("===== APF PICK AND PLACE =====")
        pre_pick  = self.pick_pos  + np.array([0, 0, 0.15])
        pre_place = self.place_pos + np.array([0, 0, 0.15])

        p1 = self.apf_path(np.array([0.3, 0.0, 1.3]), pre_pick)
        p2 = self.apf_path(pre_pick, pre_place)
        self.get_logger().info(f"APF path to pick: {len(p1)} pts, to place: {len(p2)} pts")

        self.move_home()
        self.gripper(0.0)
        self.move_to_pose(pre_pick[0],       pre_pick[1],       pre_pick[2],       "pre_grasp")
        self.move_to_pose(self.pick_pos[0],  self.pick_pos[1],  self.pick_pos[2],  "grasp")
        self.gripper(0.5)
        self.move_to_pose(pre_pick[0],       pre_pick[1],       pre_pick[2],       "lift")
        self.move_to_pose(pre_place[0],      pre_place[1],      pre_place[2],      "pre_place")
        self.move_to_pose(self.place_pos[0], self.place_pos[1], self.place_pos[2], "place")
        self.gripper(0.0)
        self.move_home()
        self.get_logger().info("===== DONE =====")

def main(args=None):
    rclpy.init(args=args)
    node = APFPickPlace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
