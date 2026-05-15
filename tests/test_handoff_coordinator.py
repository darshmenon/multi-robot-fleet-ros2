"""
Unit tests for handoff_coordinator.py FSM logic.

Mocks rclpy and std_msgs so no ROS2 installation is needed.
"""

import json
import sys
import types
import unittest
from unittest.mock import MagicMock, patch


# ---------------------------------------------------------------------------
# Stub out rclpy and std_msgs before importing the module under test
# ---------------------------------------------------------------------------

def _make_rclpy_stub():
    rclpy = types.ModuleType('rclpy')
    node_mod = types.ModuleType('rclpy.node')

    class Node:
        def __init__(self, name):
            self._pubs = {}
            self._subs = {}
            self._timers = []
            self._logger = MagicMock()

        def create_subscription(self, msg_type, topic, cb, qos):
            self._subs[topic] = cb
            return MagicMock()

        def create_publisher(self, msg_type, topic, qos):
            pub = MagicMock()
            self._pubs[topic] = pub
            return pub

        def create_timer(self, period, cb):
            self._timers.append(cb)
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


_make_rclpy_stub()

# Now safe to import
sys.path.insert(0, 'mobile_robots/diff_drive_robot-main/scripts')
from handoff_coordinator import (  # noqa: E402
    HandoffCoordinator,
    PICKUP_NAV, HANDOFF_NAV, ARM_PICK, DROPOFF_NAV, DONE, FAILED,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_node():
    return HandoffCoordinator()


def _str_msg(data: dict | str):
    from std_msgs.msg import String
    m = String()
    m.data = json.dumps(data) if isinstance(data, dict) else data
    return m


VALID_REQUEST = {
    'amr': 'robot1',
    'pickup': [1.0, 0.0, 0],
    'handoff_zone': [3.0, 0.0, 0],
    'dropoff': [5.0, 2.0, 0],
    'payload': 'box_A',
}


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestRequestCb(unittest.TestCase):

    def setUp(self):
        self.node = _make_node()

    def _pub(self, topic):
        return self.node._pubs[topic]

    def test_valid_request_creates_job(self):
        self.node._request_cb(_str_msg(VALID_REQUEST))
        self.assertEqual(len(self.node._jobs), 1)
        job = next(iter(self.node._jobs.values()))
        self.assertEqual(job['phase'], PICKUP_NAV)
        self.assertEqual(job['amr'], 'robot1')
        self.assertEqual(job['payload'], 'box_A')

    def test_valid_request_dispatches_goto_pickup(self):
        self.node._request_cb(_str_msg(VALID_REQUEST))
        self._pub('/mission/execute').publish.assert_called_once()
        sent = json.loads(self._pub('/mission/execute').publish.call_args[0][0].data)
        self.assertEqual(sent['type'], 'goto')
        self.assertEqual(sent['pose'], [1.0, 0.0, 0])

    def test_bad_json_ignored(self):
        from std_msgs.msg import String
        m = String()
        m.data = 'not json'
        self.node._request_cb(m)
        self.assertEqual(len(self.node._jobs), 0)

    def test_missing_field_ignored(self):
        bad = dict(VALID_REQUEST)
        del bad['handoff_zone']
        self.node._request_cb(_str_msg(bad))
        self.assertEqual(len(self.node._jobs), 0)

    def test_busy_amr_rejected(self):
        self.node._request_cb(_str_msg(VALID_REQUEST))
        self.node._request_cb(_str_msg(VALID_REQUEST))
        self.assertEqual(len(self.node._jobs), 1)

    def test_default_payload_label(self):
        req = dict(VALID_REQUEST)
        del req['payload']
        self.node._request_cb(_str_msg(req))
        job = next(iter(self.node._jobs.values()))
        self.assertEqual(job['payload'], 'payload')


class TestMissionCb(unittest.TestCase):

    def setUp(self):
        self.node = _make_node()
        self.node._request_cb(_str_msg(VALID_REQUEST))
        self.job_id = next(iter(self.node._jobs))

    def _nav_done(self, robot='robot1'):
        self.node._mission_cb(_str_msg({'robot': robot, 'state': 'NAVIGATING'}))
        self.node._mission_cb(_str_msg({'robot': robot, 'state': 'DONE'}))

    def test_pickup_nav_done_transitions_to_handoff_nav(self):
        self._nav_done()
        self.assertEqual(self.node._jobs[self.job_id]['phase'], HANDOFF_NAV)

    def test_handoff_nav_done_transitions_to_arm_pick(self):
        self._nav_done()  # PICKUP_NAV → HANDOFF_NAV
        self._nav_done()  # HANDOFF_NAV → ARM_PICK
        self.assertEqual(self.node._jobs[self.job_id]['phase'], ARM_PICK)

    def test_handoff_nav_done_triggers_arm(self):
        self._nav_done()
        self.node._pubs['/mission/execute'].publish.reset_mock()
        self._nav_done()
        self.node._pubs['/vla_instruction'].publish.assert_called_once()

    def test_dropoff_nav_done_closes_job(self):
        self._nav_done()   # → HANDOFF_NAV
        self._nav_done()   # → ARM_PICK
        # simulate arm completion
        self.node._arm_cb(_str_msg({'status': 'completed'}))
        self._nav_done()   # → DONE
        self.assertEqual(self.node._jobs[self.job_id]['phase'], DONE)
        self.assertNotIn('robot1', self.node._amr_job)

    def test_unknown_robot_ignored(self):
        self.node._mission_cb(_str_msg({'robot': 'ghost', 'state': 'DONE'}))
        self.assertEqual(self.node._jobs[self.job_id]['phase'], PICKUP_NAV)

    def test_non_edge_state_no_transition(self):
        # DONE without prior NAVIGATING should not advance phase
        self.node._mission_cb(_str_msg({'robot': 'robot1', 'state': 'DONE'}))
        self.assertEqual(self.node._jobs[self.job_id]['phase'], PICKUP_NAV)

    def test_pickup_nav_done_sends_goto_handoff_zone(self):
        self._nav_done()
        calls = self.node._pubs['/mission/execute'].publish.call_args_list
        last_pose = json.loads(calls[-1][0][0].data)['pose']
        self.assertEqual(last_pose, VALID_REQUEST['handoff_zone'])


class TestArmCb(unittest.TestCase):

    def setUp(self):
        self.node = _make_node()
        self.node._request_cb(_str_msg(VALID_REQUEST))
        self.job_id = next(iter(self.node._jobs))
        # Advance to ARM_PICK
        for _ in range(2):
            self.node._mission_cb(_str_msg({'robot': 'robot1', 'state': 'NAVIGATING'}))
            self.node._mission_cb(_str_msg({'robot': 'robot1', 'state': 'DONE'}))

    def test_arm_completed_transitions_to_dropoff_nav(self):
        self.node._arm_cb(_str_msg({'status': 'completed'}))
        self.assertEqual(self.node._jobs[self.job_id]['phase'], DROPOFF_NAV)

    def test_arm_completed_dispatches_goto_dropoff(self):
        self.node._pubs['/mission/execute'].publish.reset_mock()
        self.node._arm_cb(_str_msg({'status': 'completed'}))
        sent = json.loads(self.node._pubs['/mission/execute'].publish.call_args[0][0].data)
        self.assertEqual(sent['pose'], VALID_REQUEST['dropoff'])

    def test_arm_non_completed_status_ignored(self):
        self.node._arm_cb(_str_msg({'status': 'running'}))
        self.assertEqual(self.node._jobs[self.job_id]['phase'], ARM_PICK)

    def test_arm_bad_json_ignored(self):
        from std_msgs.msg import String
        m = String()
        m.data = '{'
        self.node._arm_cb(m)
        self.assertEqual(self.node._jobs[self.job_id]['phase'], ARM_PICK)


class TestCloseJob(unittest.TestCase):

    def setUp(self):
        self.node = _make_node()
        self.node._request_cb(_str_msg(VALID_REQUEST))
        self.job_id = next(iter(self.node._jobs))

    def test_close_success(self):
        self.node._close_job(self.job_id, 'robot1', success=True)
        self.assertEqual(self.node._jobs[self.job_id]['phase'], DONE)
        self.assertNotIn('robot1', self.node._amr_job)

    def test_close_failure(self):
        self.node._close_job(self.job_id, 'robot1', success=False)
        self.assertEqual(self.node._jobs[self.job_id]['phase'], FAILED)
        self.assertNotIn('robot1', self.node._amr_job)

    def test_amr_rebookable_after_close(self):
        self.node._close_job(self.job_id, 'robot1', success=True)
        self.node._request_cb(_str_msg(VALID_REQUEST))
        self.assertEqual(len(self.node._jobs), 2)


class TestPublishState(unittest.TestCase):

    def test_publishes_all_jobs(self):
        node = _make_node()
        node._request_cb(_str_msg(VALID_REQUEST))
        req2 = dict(VALID_REQUEST, amr='robot2')
        node._request_cb(_str_msg(req2))
        node._publish_state()
        pub = node._pubs['/handoff/state']
        pub.publish.assert_called_once()
        snapshot = json.loads(pub.publish.call_args[0][0].data)
        self.assertEqual(len(snapshot), 2)
        amrs = {s['amr'] for s in snapshot}
        self.assertEqual(amrs, {'robot1', 'robot2'})

    def test_published_fields(self):
        node = _make_node()
        node._request_cb(_str_msg(VALID_REQUEST))
        node._publish_state()
        snapshot = json.loads(node._pubs['/handoff/state'].publish.call_args[0][0].data)
        entry = snapshot[0]
        for key in ('job_id', 'phase', 'amr', 'payload'):
            self.assertIn(key, entry)


if __name__ == '__main__':
    unittest.main()
