#!/usr/bin/env python3
import time

from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup as MoveGroupAction
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

_ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

_NAMED_POSES = {
    "home":  [0.0, -1.6663, 0.0, 0.0, 0.0, 0.0],
    "ready": [0.0, -1.6663, 0.0, 0.0, 0.0, 1.0],
}

_GRIPPER_OPEN       = 0.0
_GRIPPER_CLOSED     = 0.8
_GRIPPER_HALF       = 0.60
_GRIPPER_MAX_EFFORT = 20.0


class MotionExecutor:
    def __init__(self, node: Node):
        self._node = node
        self._move_client = ActionClient(node, MoveGroupAction, "/move_action")
        self._gripper_client = ActionClient(
            node, GripperCommand, "/gripper_controller/gripper_cmd"
        )
        self._ik_client = node.create_client(GetPositionIK, "/compute_ik")

    def wait_for_servers(self, timeout: float = 15.0) -> bool:
        deadline = time.time() + timeout
        for name, client in [
            ("/move_action", self._move_client),
            ("/gripper_controller/gripper_cmd", self._gripper_client),
        ]:
            remaining = deadline - time.time()
            if remaining <= 0 or not client.wait_for_server(timeout_sec=remaining):
                self._node.get_logger().error(f"Timeout waiting for {name}")
                return False
        remaining = deadline - time.time()
        if remaining > 0:
            self._ik_client.wait_for_service(timeout_sec=remaining)
        return True

    def _move_to_joint_values(
        self, group: str, vals: list, timeout: float = 10.0
    ) -> bool:
        req = MotionPlanRequest()
        req.group_name = group
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "PTP"
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        req.num_planning_attempts = 1

        c = Constraints()
        for jname, value in zip(_ARM_JOINTS, vals):
            jc = JointConstraint()
            jc.joint_name = jname
            jc.position = float(value)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints.append(c)
        return self._send_move_group(req, timeout)

    def move_to_named_pose(self, group: str, name: str, timeout: float = 15.0) -> bool:
        vals = _NAMED_POSES.get(name)
        if vals is None:
            self._node.get_logger().error(f"Unknown named pose: {name}")
            return False
        return self._move_to_joint_values(group, vals, timeout)

    def move_to_pose(self, pose: PoseStamped, timeout: float = 20.0) -> bool:
        req = MotionPlanRequest()
        req.group_name = "arm"
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "PTP"
        req.allowed_planning_time = 15.0
        req.max_velocity_scaling_factor = 0.2
        req.max_acceleration_scaling_factor = 0.2
        req.num_planning_attempts = 1

        c = Constraints()

        pc = PositionConstraint()
        pc.header.frame_id = pose.header.frame_id
        pc.link_name = "tool0"
        region = BoundingVolume()
        sp = SolidPrimitive()
        sp.type = SolidPrimitive.SPHERE
        sp.dimensions = [0.005]
        region.primitives.append(sp)
        cp = Pose()
        cp.position = pose.pose.position
        region.primitive_poses.append(cp)
        pc.constraint_region = region
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = pose.header.frame_id
        oc.link_name = "tool0"
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0
        c.orientation_constraints.append(oc)

        req.goal_constraints.append(c)
        return self._send_move_group(req, timeout)

    def _compute_ik(
        self, pose: PoseStamped, group: str = "arm", timeout: float = 5.0
    ):
        if not self._ik_client.service_is_ready():
            return None
        req = GetPositionIK.Request()
        req.ik_request.group_name = group
        req.ik_request.pose_stamped = pose
        req.ik_request.timeout.sec = int(timeout)
        req.ik_request.timeout.nanosec = int((timeout % 1) * 1e9)

        future = self._ik_client.call_async(req)
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                return None
            time.sleep(0.05)

        result = future.result()
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        positions = []
        for jname in _ARM_JOINTS:
            try:
                idx = result.solution.joint_state.name.index(jname)
                positions.append(result.solution.joint_state.position[idx])
            except ValueError:
                return None
        return positions

    def open_gripper(self, timeout: float = 8.0) -> bool:
        return self._send_gripper(_GRIPPER_OPEN, timeout)

    def close_gripper(self, timeout: float = 8.0) -> bool:
        return self._send_gripper(_GRIPPER_CLOSED, timeout)

    def half_close_gripper(self, timeout: float = 8.0) -> bool:
        return self._send_gripper(_GRIPPER_HALF, timeout)

    def _send_gripper(self, position: float, timeout: float) -> bool:
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = _GRIPPER_MAX_EFFORT

        future = self._gripper_client.send_goal_async(goal)
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                return False
            time.sleep(0.05)

        handle = future.result()
        if not handle.accepted:
            return False

        result_future = handle.get_result_async()
        deadline = time.time() + timeout
        while not result_future.done():
            if time.time() > deadline:
                return False
            time.sleep(0.05)

        return True  # stall on close is normal

    def _send_move_group(self, req: MotionPlanRequest, timeout: float) -> bool:
        goal = MoveGroupAction.Goal()
        goal.request = req
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False

        future = self._move_client.send_goal_async(goal)
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                self._node.get_logger().error("send_goal timeout")
                return False
            time.sleep(0.05)

        handle = future.result()
        if not handle.accepted:
            self._node.get_logger().error("goal rejected")
            return False

        result_future = handle.get_result_async()
        deadline = time.time() + timeout
        while not result_future.done():
            if time.time() > deadline:
                self._node.get_logger().error("result timeout")
                return False
            time.sleep(0.05)

        result = result_future.result().result
        ok = result.error_code.val == MoveItErrorCodes.SUCCESS
        if not ok:
            self._node.get_logger().warn(
                f"MoveGroup error code: {result.error_code.val}"
            )
        return ok
