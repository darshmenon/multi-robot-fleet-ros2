"""
Unit tests for llm_planner_node.py.

Stubs out rclpy, anthropic, and MotionExecutor so no ROS 2 or LLM credentials
are required.
"""

import json
import sys
import types
import unittest
from unittest.mock import MagicMock, patch, call


# ---------------------------------------------------------------------------
# Minimal rclpy / std_msgs / geometry_msgs stubs
# ---------------------------------------------------------------------------

def _stub_rclpy():
    rclpy = types.ModuleType('rclpy')
    node_mod = types.ModuleType('rclpy.node')

    class Node:
        def __init__(self, name):
            self._pubs = {}
            self._subs = {}
            self._services = {}
            self._logger = MagicMock()
            self._params = {}

        def declare_parameter(self, name, default):
            self._params[name] = default

        def get_parameter(self, name):
            m = MagicMock()
            m.value = self._params.get(name, '')
            return m

        def create_subscription(self, msg_type, topic, cb, qos):
            self._subs[topic] = cb
            return MagicMock()

        def create_publisher(self, msg_type, topic, qos):
            pub = MagicMock()
            self._pubs[topic] = pub
            return pub

        def create_service(self, srv_type, name, cb):
            self._services[name] = cb
            return MagicMock()

        def get_logger(self):
            return self._logger

    node_mod.Node = Node
    rclpy.node = node_mod
    rclpy.init = MagicMock()
    rclpy.spin = MagicMock()
    rclpy.shutdown = MagicMock()
    sys.modules['rclpy'] = rclpy
    sys.modules['rclpy.node'] = node_mod

    std_msgs = types.ModuleType('std_msgs')
    msg_mod = types.ModuleType('std_msgs.msg')

    class String:
        def __init__(self):
            self.data = ''

    msg_mod.String = String
    std_msgs.msg = msg_mod
    sys.modules['std_msgs'] = std_msgs
    sys.modules['std_msgs.msg'] = msg_mod

    geo_msgs = types.ModuleType('geometry_msgs')
    geo_msg_mod = types.ModuleType('geometry_msgs.msg')

    class PoseStamped:
        def __init__(self):
            self.header = MagicMock()
            self.header.frame_id = ''
            self.pose = MagicMock()
            self.pose.position = MagicMock()
            self.pose.orientation = MagicMock()

    geo_msg_mod.PoseStamped = PoseStamped
    geo_msgs.msg = geo_msg_mod
    sys.modules['geometry_msgs'] = geo_msgs
    sys.modules['geometry_msgs.msg'] = geo_msg_mod


def _stub_ur_interfaces():
    ur_iface = types.ModuleType('ur_interfaces')
    srv_mod = types.ModuleType('ur_interfaces.srv')

    class ExecuteCommand:
        class Request:
            command = ''
        class Response:
            success = False
            message = ''

    srv_mod.ExecuteCommand = ExecuteCommand
    ur_iface.srv = srv_mod
    sys.modules['ur_interfaces'] = ur_iface
    sys.modules['ur_interfaces.srv'] = srv_mod


_stub_rclpy()
_stub_ur_interfaces()

# Stub motion_executor inside the package
class _FakeMotionExecutor:
    def __init__(self, node):
        self.node = node
        self.calls = []

    def wait_for_servers(self, timeout=15.0):
        return True

    def move_to_named_pose(self, group, name, timeout=15.0):
        self.calls.append(('move_to_named_pose', group, name))
        return True

    def move_to_pose(self, pose, timeout=20.0):
        self.calls.append(('move_to_pose', pose))
        return True

    def open_gripper(self, timeout=8.0):
        self.calls.append(('open_gripper',))
        return True

    def close_gripper(self, timeout=8.0):
        self.calls.append(('close_gripper',))
        return True

    def half_close_gripper(self, timeout=8.0):
        self.calls.append(('half_close_gripper',))
        return True


# Stub motion_executor and anthropic before importing the module under test
_motion_stub = types.ModuleType('ur_llm_planner.motion_executor')
_motion_stub.MotionExecutor = _FakeMotionExecutor
sys.modules['ur_llm_planner.motion_executor'] = _motion_stub

_anthropic_stub = types.ModuleType('anthropic')
_anthropic_stub.Anthropic = MagicMock
sys.modules['anthropic'] = _anthropic_stub

# Import from the real package on disk
sys.path.insert(0, 'manipulation/ur_llm_planner')
# Clear any bare stub so Python uses the real package from the path
sys.modules.pop('ur_llm_planner', None)
from ur_llm_planner.llm_planner_node import LLMPlannerNode  # noqa: E402


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_SIMPLE_PLAN = [
    {'action': 'move_to_named_pose', 'group': 'arm', 'name': 'ready'},
    {'action': 'open_gripper'},
    {'action': 'move_to_pose', 'frame_id': 'base_link',
     'x': 0.4, 'y': 0.0, 'z': 0.3, 'qx': 0.0, 'qy': 0.707, 'qz': 0.0, 'qw': 0.707},
    {'action': 'close_gripper'},
    {'action': 'move_to_named_pose', 'group': 'arm', 'name': 'home'},
]


def _make_node():
    node = LLMPlannerNode.__new__(LLMPlannerNode)
    # Call Node.__init__ manually (our stub)
    from rclpy.node import Node as StubNode
    StubNode.__init__(node, 'llm_planner_node')
    node._params = {
        'model': 'claude-haiku-4-5-20251001',
        'anthropic_api_key': '',
    }
    node._model = 'claude-haiku-4-5-20251001'
    node._motion = _FakeMotionExecutor(node)
    node._busy = False
    import threading
    node._lock = threading.Lock()
    node._feedback_pub = node._pubs.get('/vla/task_feedback', MagicMock())
    return node


def _fake_claude(plan):
    client = MagicMock()
    response = MagicMock()
    response.content = [MagicMock()]
    response.content[0].text = json.dumps(plan)
    client.messages.create.return_value = response
    return client


# ---------------------------------------------------------------------------
# Tests: _plan
# ---------------------------------------------------------------------------

class TestPlan(unittest.TestCase):

    def setUp(self):
        self.node = _make_node()

    def test_returns_parsed_plan(self):
        self.node._claude = _fake_claude(_SIMPLE_PLAN)
        plan = self.node._plan('pick the box')
        self.assertEqual(plan, _SIMPLE_PLAN)

    def test_strips_markdown_fences(self):
        client = MagicMock()
        resp = MagicMock()
        resp.content = [MagicMock()]
        resp.content[0].text = '```json\n' + json.dumps(_SIMPLE_PLAN) + '\n```'
        client.messages.create.return_value = resp
        self.node._claude = client
        plan = self.node._plan('pick the box')
        self.assertEqual(plan, _SIMPLE_PLAN)

    def test_returns_none_on_bad_json(self):
        client = MagicMock()
        resp = MagicMock()
        resp.content = [MagicMock()]
        resp.content[0].text = 'not json'
        client.messages.create.return_value = resp
        self.node._claude = client
        plan = self.node._plan('pick the box')
        self.assertIsNone(plan)

    def test_returns_none_when_no_client(self):
        self.node._claude = None
        plan = self.node._plan('pick the box')
        self.assertIsNone(plan)

    def test_returns_none_on_api_exception(self):
        client = MagicMock()
        client.messages.create.side_effect = RuntimeError('network error')
        self.node._claude = client
        plan = self.node._plan('pick the box')
        self.assertIsNone(plan)

    def test_passes_command_to_llm(self):
        self.node._claude = _fake_claude(_SIMPLE_PLAN)
        self.node._plan('grasp the cylinder')
        args, kwargs = self.node._claude.messages.create.call_args
        msgs = kwargs.get('messages') or args[0] if args else kwargs['messages']
        self.assertIn('grasp the cylinder', json.dumps(kwargs))


# ---------------------------------------------------------------------------
# Tests: _run_step
# ---------------------------------------------------------------------------

class TestRunStep(unittest.TestCase):

    def setUp(self):
        self.node = _make_node()

    def test_move_to_named_pose(self):
        ok = self.node._run_step({'action': 'move_to_named_pose', 'group': 'arm', 'name': 'home'})
        self.assertTrue(ok)
        self.assertIn(('move_to_named_pose', 'arm', 'home'), self.node._motion.calls)

    def test_move_to_pose(self):
        step = {'action': 'move_to_pose', 'frame_id': 'base_link',
                'x': 0.4, 'y': 0.0, 'z': 0.3, 'qx': 0.0, 'qy': 0.707, 'qz': 0.0, 'qw': 0.707}
        ok = self.node._run_step(step)
        self.assertTrue(ok)
        name, pose = self.node._motion.calls[0]
        self.assertEqual(name, 'move_to_pose')
        self.assertEqual(pose.pose.position.x, 0.4)
        self.assertEqual(pose.header.frame_id, 'base_link')

    def test_open_gripper(self):
        ok = self.node._run_step({'action': 'open_gripper'})
        self.assertTrue(ok)
        self.assertIn(('open_gripper',), self.node._motion.calls)

    def test_close_gripper(self):
        ok = self.node._run_step({'action': 'close_gripper'})
        self.assertTrue(ok)
        self.assertIn(('close_gripper',), self.node._motion.calls)

    def test_half_close_gripper(self):
        ok = self.node._run_step({'action': 'half_close_gripper'})
        self.assertTrue(ok)
        self.assertIn(('half_close_gripper',), self.node._motion.calls)

    def test_unknown_action_returns_false(self):
        ok = self.node._run_step({'action': 'fly_to_moon'})
        self.assertFalse(ok)

    def test_move_to_pose_default_quaternion(self):
        step = {'action': 'move_to_pose', 'x': 0.3, 'y': 0.1, 'z': 0.5}
        ok = self.node._run_step(step)
        self.assertTrue(ok)
        _, pose = self.node._motion.calls[0]
        self.assertEqual(pose.pose.orientation.w, 1.0)


# ---------------------------------------------------------------------------
# Tests: _execute
# ---------------------------------------------------------------------------

class TestExecute(unittest.TestCase):

    def setUp(self):
        self.node = _make_node()
        self.node._claude = _fake_claude(_SIMPLE_PLAN)

    def test_success_returns_true(self):
        ok, msg = self.node._execute('pick the box')
        self.assertTrue(ok)
        self.assertEqual(msg, 'Done')

    def test_runs_all_steps(self):
        self.node._execute('pick the box')
        action_names = [c[0] for c in self.node._motion.calls]
        self.assertIn('move_to_named_pose', action_names)
        self.assertIn('open_gripper', action_names)
        self.assertIn('move_to_pose', action_names)
        self.assertIn('close_gripper', action_names)

    def test_returns_false_on_plan_failure(self):
        self.node._claude = None
        ok, msg = self.node._execute('pick the box')
        self.assertFalse(ok)

    def test_stops_on_first_step_failure(self):
        self.node._motion.open_gripper = MagicMock(return_value=False)
        ok, _ = self.node._execute('pick the box')
        self.assertFalse(ok)
        # move_to_pose should not have been called
        action_names = [c[0] for c in self.node._motion.calls]
        self.assertNotIn('move_to_pose', action_names)

    def test_servers_not_ready_returns_false(self):
        self.node._motion.wait_for_servers = MagicMock(return_value=False)
        ok, msg = self.node._execute('pick the box')
        self.assertFalse(ok)
        self.assertIn('not ready', msg)

    def test_busy_flag_cleared_after_success(self):
        self.node._execute('pick the box')
        self.assertFalse(self.node._busy)

    def test_busy_flag_cleared_after_failure(self):
        self.node._claude = None
        self.node._execute('pick the box')
        self.assertFalse(self.node._busy)


# ---------------------------------------------------------------------------
# Tests: _instruction_cb and _run (feedback publishing)
# ---------------------------------------------------------------------------

class TestInstructionCb(unittest.TestCase):

    def setUp(self):
        self.node = _make_node()
        self.node._claude = _fake_claude(_SIMPLE_PLAN)
        self.node._feedback_pub = MagicMock()

    def _make_str(self, text):
        from std_msgs.msg import String
        m = String()
        m.data = text
        return m

    def test_run_publishes_completed_on_success(self):
        self.node._run('pick the box')
        self.node._feedback_pub.publish.assert_called_once()
        payload = json.loads(self.node._feedback_pub.publish.call_args[0][0].data)
        self.assertEqual(payload['status'], 'completed')

    def test_run_publishes_failed_on_failure(self):
        self.node._claude = None
        self.node._run('pick the box')
        payload = json.loads(self.node._feedback_pub.publish.call_args[0][0].data)
        self.assertEqual(payload['status'], 'failed')

    def test_instruction_cb_ignored_when_busy(self):
        self.node._busy = True
        cb_called = []
        self.node._run = lambda cmd: cb_called.append(cmd)
        self.node._instruction_cb(self._make_str('pick the box'))
        import time; time.sleep(0.05)
        self.assertEqual(cb_called, [])


# ---------------------------------------------------------------------------
# Tests: _service_cb
# ---------------------------------------------------------------------------

class TestServiceCb(unittest.TestCase):

    def setUp(self):
        self.node = _make_node()
        self.node._claude = _fake_claude(_SIMPLE_PLAN)

    def _make_request(self, command):
        from ur_interfaces.srv import ExecuteCommand
        req = ExecuteCommand.Request()
        req.command = command
        return req

    def test_service_returns_success(self):
        from ur_interfaces.srv import ExecuteCommand
        req = self._make_request('pick the box')
        resp = ExecuteCommand.Response()
        result = self.node._service_cb(req, resp)
        self.assertTrue(result.success)
        self.assertEqual(result.message, 'Done')

    def test_service_returns_failure(self):
        from ur_interfaces.srv import ExecuteCommand
        self.node._claude = None
        req = self._make_request('pick the box')
        resp = ExecuteCommand.Response()
        result = self.node._service_cb(req, resp)
        self.assertFalse(result.success)


if __name__ == '__main__':
    unittest.main()
