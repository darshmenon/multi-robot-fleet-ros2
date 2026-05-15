#!/usr/bin/env python3
"""
handoff_coordinator.py — Orchestrates AMR-to-arm payload handoff.

FSM per job:
  PICKUP_NAV  → AMR navigates to pickup location
  HANDOFF_NAV → AMR navigates to handoff zone (arm reach)
  ARM_PICK    → arm picks payload from AMR; AMR holds position
  DROPOFF_NAV → AMR navigates to dropoff
  DONE / FAILED

Topics
------
  Sub:  /handoff/request    (String JSON) — trigger a new handoff job
  Sub:  /mission/state      (String JSON) — AMR mission phase from mission_server
  Sub:  /vla/task_feedback  (String JSON) — arm completion from VLA coordinator
  Pub:  /mission/execute    (String JSON) — dispatch AMR goto missions
  Pub:  /vla_instruction    (String)      — trigger arm pick
  Pub:  /handoff/state      (String JSON) — active job states at 1 Hz

Request JSON
------------
  {
    "amr":          "robot1",
    "pickup":       [x, y, yaw_deg],
    "handoff_zone": [x, y, yaw_deg],
    "dropoff":      [x, y, yaw_deg],
    "payload":      "box_A"
  }

Quick test (daemon must be running):
  ros2 topic pub --once /handoff/request std_msgs/msg/String \\
    '{data: "{\"amr\":\"robot1\",\"pickup\":[1.0,0.0,0],\\
              \"handoff_zone\":[3.0,0.0,0],\\
              \"dropoff\":[5.0,2.0,0],\"payload\":\"box_A\"}"}'

  ros2 topic echo /handoff/state
"""

import json
import threading
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

PICKUP_NAV  = 'PICKUP_NAV'
HANDOFF_NAV = 'HANDOFF_NAV'
ARM_PICK    = 'ARM_PICK'
DROPOFF_NAV = 'DROPOFF_NAV'
DONE        = 'DONE'
FAILED      = 'FAILED'


class HandoffCoordinator(Node):
    def __init__(self):
        super().__init__('handoff_coordinator')

        self._lock = threading.Lock()
        # job_id → job dict
        self._jobs: dict[str, dict] = {}
        # amr_ns → job_id (prevents double-booking an AMR)
        self._amr_job: dict[str, str] = {}

        self.create_subscription(String, '/handoff/request',   self._request_cb,  10)
        self.create_subscription(String, '/mission/state',     self._mission_cb,  10)
        self.create_subscription(String, '/vla/task_feedback', self._arm_cb,      10)

        self._mission_pub = self.create_publisher(String, '/mission/execute', 10)
        self._arm_pub     = self.create_publisher(String, '/vla_instruction', 10)
        self._state_pub   = self.create_publisher(String, '/handoff/state',   10)

        self.create_timer(1.0, self._publish_state)

        self.get_logger().info('HandoffCoordinator ready — listening on /handoff/request')

    # ── Job intake ────────────────────────────────────────────────────────────

    def _request_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Bad request JSON: {e}')
            return

        for field in ('amr', 'pickup', 'handoff_zone', 'dropoff'):
            if field not in data:
                self.get_logger().error(f'Missing required field: {field!r}')
                return

        amr = data['amr']
        with self._lock:
            if amr in self._amr_job:
                self.get_logger().warn(
                    f'AMR {amr!r} busy with job {self._amr_job[amr]!r} — ignoring.'
                )
                return

            job_id = str(uuid.uuid4())[:8]
            self._jobs[job_id] = {
                'phase':        PICKUP_NAV,
                'amr':          amr,
                'pickup':       data['pickup'],
                'handoff_zone': data['handoff_zone'],
                'dropoff':      data['dropoff'],
                'payload':      data.get('payload', 'payload'),
                'prev_state':   '',
            }
            self._amr_job[amr] = job_id

        self.get_logger().info(
            f'[{job_id}] New handoff: AMR={amr} payload={data.get("payload", "?")} '
            f'pickup→handoff_zone→dropoff'
        )
        self._goto(job_id, amr, data['pickup'], 'pickup')

    # ── AMR state transitions ─────────────────────────────────────────────────

    def _mission_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        amr   = data.get('robot', '')
        state = data.get('state', '')

        with self._lock:
            job_id = self._amr_job.get(amr)
            if not job_id or job_id not in self._jobs:
                return
            job   = self._jobs[job_id]
            prev  = job['prev_state']
            job['prev_state'] = state
            phase = job['phase']
            # snapshot immutable fields for use outside the lock
            handoff_zone = job['handoff_zone']
            dropoff      = job['dropoff']
            payload      = job['payload']

        # Only act on NAVIGATING → DONE edge
        if prev != 'NAVIGATING' or state != 'DONE':
            return

        if phase == PICKUP_NAV:
            self.get_logger().info(f'[{job_id}] Pickup reached — heading to handoff zone.')
            with self._lock:
                self._jobs[job_id]['phase'] = HANDOFF_NAV
            self._goto(job_id, amr, handoff_zone, 'handoff_zone')

        elif phase == HANDOFF_NAV:
            self.get_logger().info(f'[{job_id}] Handoff zone reached — triggering arm.')
            with self._lock:
                self._jobs[job_id]['phase'] = ARM_PICK
            self._trigger_arm(job_id, payload)

        elif phase == DROPOFF_NAV:
            self.get_logger().info(f'[{job_id}] Dropoff reached — handoff complete.')
            self._close_job(job_id, amr, success=True)

    # ── Arm completion ────────────────────────────────────────────────────────

    def _arm_cb(self, msg: String):
        try:
            fb = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if fb.get('status') != 'completed':
            return

        with self._lock:
            job_id = next(
                (jid for jid, j in self._jobs.items() if j['phase'] == ARM_PICK),
                None,
            )
            if not job_id:
                return
            job           = self._jobs[job_id]
            job['phase']  = DROPOFF_NAV
            amr           = job['amr']
            dropoff       = job['dropoff']

        self.get_logger().info(f'[{job_id}] Arm pick done — sending AMR to dropoff.')
        self._goto(job_id, amr, dropoff, 'dropoff')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _goto(self, job_id: str, amr: str, pose: list, label: str):
        payload = {'type': 'goto', 'robot': amr, 'pose': pose, 'waypoints': []}
        msg      = String()
        msg.data = json.dumps(payload)
        self._mission_pub.publish(msg)
        self.get_logger().info(f'[{job_id}] goto {label} {pose}')

    def _trigger_arm(self, job_id: str, payload_label: str):
        msg      = String()
        msg.data = f'pick the {payload_label} from the AMR and place it on the shelf'
        self._arm_pub.publish(msg)
        self.get_logger().info(f'[{job_id}] arm trigger: "{msg.data}"')

    def _close_job(self, job_id: str, amr: str, success: bool):
        with self._lock:
            if job_id in self._jobs:
                self._jobs[job_id]['phase'] = DONE if success else FAILED
            self._amr_job.pop(amr, None)
        self.get_logger().info(
            f'[{job_id}] {"DONE" if success else "FAILED"}'
        )

    def _publish_state(self):
        with self._lock:
            snapshot = [
                {
                    'job_id':  jid,
                    'phase':   j['phase'],
                    'amr':     j['amr'],
                    'payload': j['payload'],
                }
                for jid, j in self._jobs.items()
            ]
        msg      = String()
        msg.data = json.dumps(snapshot)
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HandoffCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
