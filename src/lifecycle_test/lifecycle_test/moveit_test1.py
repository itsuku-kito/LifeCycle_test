#!/usr/bin/env python3
# coding: UTF-8

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.duration import Duration


class MovePlanner(Node):
    """
    MoveIt2の MoveGroup ActionClient を用いた制御ノード
    """

    def __init__(self):
        super().__init__('move_planner_move_action')

        # 現在のジョイント状態を保持
        self.currentpositions = [0.0] * 19
        self.currentpositions_lock = False

        # MoveGroupアクションクライアント
        self.client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info("Waiting for MoveGroup action server (/move_action)...")
        self.client.wait_for_server()
        self.get_logger().info("MoveGroup action server connected.")

        # joint_states購読
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.update_position,
            10
        )

    # ====== 共通送信処理 ======
    def send_goal(self, goal_msg):
        """MoveGroupアクションを送信して結果を待つ"""
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveGroup.")
            return False

        self.get_logger().info("Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val == 1:
            self.get_logger().info("MoveGroup executed successfully.")
            return True
        else:
            self.get_logger().warn(f"MoveGroup execution failed: {result.error_code.val}")
            return False

    # ====== 上半身ジョイント動作 ======
    def joint_value_upper_body(self, *joints):
        goal = self.create_both_arm_goal(*joints)
        return self.send_goal(goal)

    def create_both_arm_goal(self, *joints):
        goal = MoveGroup.Goal()
        request = MotionPlanRequest()

        request.group_name = "both_arms"
        request.num_planning_attempts = 1
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5

        # ゴール条件（各ジョイントの制約）
        constraints = Constraints()
        joint_names = [
            "left_joint1", "left_joint2", "left_joint3", "left_joint4", "left_joint5", "left_joint6", "left_joint7",
            "right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5", "right_joint6", "right_joint7"
        ]

        for i, name in enumerate(joint_names):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = joints[i]
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        request.goal_constraints.append(constraints)
        goal.request = request
        return goal

    # ===== joint_states更新 =====
    def update_position(self, msg):
        if not self.currentpositions_lock:
            self.currentpositions_lock = True
            self.currentpositions = msg.position
            self.currentpositions_lock = False

    # ===== 初期ポーズ =====
    def initial_pose_upper_body(self):
        return self.joint_value_upper_body(
            0.00, -1.20, 0.00, -1.00, 0.00, 0.00, 0.00,
            0.00, -1.20, 0.00, -1.00, 0.00, 0.00, 0.00
        )


# ================================
#              MAIN
# ================================
def main(args=None):
    rclpy.init(args=args)
    node = MovePlanner()

    node.get_logger().info("Executing initial_pose_upper_body...")

    success = node.initial_pose_upper_body()

    if success:
        node.get_logger().info("✔ Successfully moved to initial upper body pose.")
    else:
        node.get_logger().error("❌ Failed to move to initial pose.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
