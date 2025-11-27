#!/usr/bin/env python3
# coding: UTF-8

import rclpy
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from sensor_msgs.msg import JointState

class MovePlannerLifecycle(LifecycleNode):
    def __init__(self):
        super().__init__('move_planner_lifecycle')

        # ジョイント状態
        self.currentpositions = [0.0] * 19
        self.currentpositions_lock = False

        # MoveGroupアクションクライアント
        self.client = ActionClient(self, MoveGroup, '/move_action')

        self.sub = None

    # ===== ライフサイクルコールバック =====
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring node...')
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.update_position,
            10
        )
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating node...')
        self.get_logger().info("Waiting for MoveGroup action server...")
        self.client.wait_for_server()
        self.get_logger().info("MoveGroup action server connected.")

        # 非同期で初期ポーズ送信
        self.send_initial_pose_async()
        return TransitionCallbackReturn.SUCCESS

    # ===== アクション送信を非同期化 =====
    def send_initial_pose_async(self):
        goal = self.create_both_arm_goal(
            0.00,-1.20,0.00,-1.00,0.00,0.00,0.00,
            0.00,-1.20,0.00,-1.00,0.00,0.00,0.00
        )
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self.initial_pose_sent_callback)

    def initial_pose_sent_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Initial pose goal rejected by MoveGroup.")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.initial_pose_result_callback)

    def initial_pose_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info("Initial pose executed successfully.")
        else:
            self.get_logger().warn(f"Initial pose execution failed: {result.error_code.val}")

    # ===== 既存処理 =====
    def create_both_arm_goal(self, *joints):
        from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
        goal = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = "both_arms"
        request.num_planning_attempts = 1
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = 0.5
        request.max_acceleration_scaling_factor = 0.5

        constraints = Constraints()
        joint_names = [
            "left_joint1","left_joint2","left_joint3","left_joint4","left_joint5","left_joint6","left_joint7",
            "right_joint1","right_joint2","right_joint3","right_joint4","right_joint5","right_joint6","right_joint7"
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

    def update_position(self, msg):
        if not self.currentpositions_lock:
            self.currentpositions_lock = True
            self.currentpositions = msg.position
            self.currentpositions_lock = False


def main(args=None):
    rclpy.init(args=args)
    node = MovePlannerLifecycle()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
