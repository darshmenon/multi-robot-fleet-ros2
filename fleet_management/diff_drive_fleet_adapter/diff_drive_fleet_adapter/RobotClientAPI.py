"""
RobotAPI for diff_drive_robot.

Same interface as rmf_demos_fleet_adapter's RobotAPI (navigate/stop/
start_activity/toggle_teleop/toggle_attach/get_data), but backed directly by
ROS 2 (Nav2 NavigateToPose action + tf) instead of a REST fleet manager,
since diff_drive_robot already exposes clean namespaced Nav2 topics/actions
and there is nothing to bridge.

Each robot's pose comes from the map -> {robot_name}/base_link transform,
which diff_drive_robot's tf_map_relay.py node makes available on the global
/tf topic. diff_drive_robot has no battery topic, so battery_soc is a fixed
stub (see RobotUpdateData).
"""
import enum
import math
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer
from tf2_ros import ConnectivityException
from tf2_ros import ExtrapolationException
from tf2_ros import LookupException
from tf2_ros import TransformListener


class RobotAPIResult(enum.IntEnum):
    SUCCESS = 0
    """The request was successful"""

    RETRY = 1
    """The client failed to connect but might succeed if you try again"""

    IMPOSSIBLE = 2
    """The client connected but something about the request is impossible"""


def _make_pose_stamped(pose, frame_id: str) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(pose[0])
    msg.pose.position.y = float(pose[1])
    yaw = float(pose[2]) if len(pose) > 2 else 0.0
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


class RobotAPI:
    def __init__(self, node: Node, map_name: str = 'L1'):
        self.node = node
        self.map_name = map_name
        self.debug = False
        self._lock = threading.Lock()
        self._clients: dict[str, ActionClient] = {}
        self._goal_handles: dict[str, object] = {}
        self._last_completed_cmd: dict[str, int] = {}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

    def check_connection(self):
        """Return True if connection to the robot API server is successful."""
        return True

    def _client(self, robot_name: str) -> ActionClient:
        if robot_name not in self._clients:
            self._clients[robot_name] = ActionClient(
                self.node, NavigateToPose, f'/{robot_name}/navigate_to_pose'
            )
            self._last_completed_cmd[robot_name] = 0
        return self._clients[robot_name]

    def navigate(
        self,
        robot_name: str,
        cmd_id: int,
        pose,
        map_name: str,
        speed_limit=0.0,
    ):
        """
        Request the robot to navigate to pose:[x,y,theta] via Nav2.

        Returns True once the goal has been sent to the action server
        (acceptance/completion are tracked asynchronously via cmd_id).
        """
        assert len(pose) > 2
        client = self._client(robot_name)
        if not client.wait_for_server(timeout_sec=1.0):
            print(f'Nav2 action server not available for {robot_name}')
            return False

        goal = NavigateToPose.Goal()
        goal.pose = _make_pose_stamped(pose, 'map')

        def goal_response_cb(future):
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                return
            with self._lock:
                self._goal_handles[robot_name] = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda f: self._on_result(robot_name, cmd_id)
            )

        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(goal_response_cb)
        return True

    def _on_result(self, robot_name: str, cmd_id: int):
        with self._lock:
            self._last_completed_cmd[robot_name] = cmd_id

    def start_activity(
        self, robot_name: str, cmd_id: int, activity: str, label: str
    ):
        """
        diff_drive_robot has no docking/cleaning activities.

        Returning IMPOSSIBLE makes the fleet adapter fall back to a plain
        navigate() call for docking requests (see fleet_adapter.py).
        """
        return RobotAPIResult.IMPOSSIBLE

    def stop(self, robot_name: str, running_cmd_id: int, stop_cmd_id: int):
        """Cancel the robot's current Nav2 goal."""
        with self._lock:
            goal_handle = self._goal_handles.get(robot_name)
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        with self._lock:
            self._last_completed_cmd[robot_name] = stop_cmd_id
        return True

    def toggle_teleop(self, robot_name: str, toggle: bool):
        """diff_drive_robot has no teleop-mode toggle; not supported."""
        return False

    def toggle_attach(self, robot_name: str, attach: bool, cmd_id: int):
        """diff_drive_robot has no cart-attach mechanism; not supported."""
        return False

    def get_data(self, robot_name: str | None = None):
        """
        Return a RobotUpdateData for one robot if a name is given.

        diff_drive_robot has no fleet-wide status endpoint, so a bulk query
        (robot_name=None) is not supported here.
        """
        if robot_name is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                f'{robot_name}/base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except (
            LookupException,
            ConnectivityException,
            ExtrapolationException,
        ) as err:
            if self.debug:
                print(f'Other error for {robot_name} in get_data: {err}')
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        with self._lock:
            last_completed = self._last_completed_cmd.get(robot_name, 0)
        return RobotUpdateData(
            robot_name, [t.x, t.y, yaw], self.map_name, last_completed
        )


class RobotUpdateData:
    """Update data for a single robot."""

    def __init__(
        self,
        robot_name: str,
        position,
        map_name: str,
        last_completed_request: int,
        battery_soc: float = 1.0,
    ):
        self.robot_name = robot_name
        self.position = position  # [x, y, yaw]
        self.map = map_name
        # diff_drive_robot has no battery topic; stubbed at full charge.
        self.battery_soc = battery_soc
        self.requires_replan = False
        self.last_request_completed = last_completed_request

    def is_command_completed(self, cmd_id):
        return self.last_request_completed == cmd_id
