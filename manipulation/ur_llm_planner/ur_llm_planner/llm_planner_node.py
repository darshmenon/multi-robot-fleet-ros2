#!/usr/bin/env python3
"""
LLM-driven motion planner for UR arm.

Listens on /vla_instruction (String) from the handoff coordinator and on the
/ur/execute_command service (ur_interfaces/ExecuteCommand).  Converts natural-
language instructions to a primitive action sequence via the Claude API, then
executes them through MotionExecutor.

Publishes JSON feedback to /vla/task_feedback so the handoff coordinator can
advance its FSM when the arm pick completes.

Required env var: ANTHROPIC_API_KEY (or pass via 'anthropic_api_key' ROS param).
"""

import json
import os
import threading

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String

from .motion_executor import MotionExecutor

try:
    import anthropic as _anthropic
    _ANTHROPIC_OK = True
except ImportError:
    _ANTHROPIC_OK = False

_SYSTEM_PROMPT = """\
You are a robot arm motion planner. Convert the instruction into a JSON array of
primitive actions. Output ONLY valid JSON — no markdown, no commentary.

Available actions
-----------------
{"action": "move_to_named_pose", "group": "arm", "name": "home"}
{"action": "move_to_named_pose", "group": "arm", "name": "ready"}
{"action": "move_to_pose", "frame_id": "base_link",
 "x": <float>, "y": <float>, "z": <float>,
 "qx": <float>, "qy": <float>, "qz": <float>, "qw": <float>}
{"action": "open_gripper"}
{"action": "close_gripper"}
{"action": "half_close_gripper"}

Rules
-----
- Always start with move_to_named_pose home or ready before approaching a target.
- Always open_gripper before reaching the pick pose.
- Return home after placing.
- Use frame_id "base_link" for all Cartesian poses.
- Quaternion (qx=0, qy=0.707, qz=0, qw=0.707) points the tool straight down.
"""


class LLMPlannerNode(Node):

    def __init__(self):
        super().__init__('llm_planner_node')

        self.declare_parameter('model', 'claude-haiku-4-5-20251001')
        self.declare_parameter('anthropic_api_key', '')

        model = self.get_parameter('model').value
        api_key = (
            self.get_parameter('anthropic_api_key').value
            or os.environ.get('ANTHROPIC_API_KEY', '')
        )

        if _ANTHROPIC_OK:
            self._claude = _anthropic.Anthropic(api_key=api_key) if api_key else None
        else:
            self._claude = None
            self.get_logger().warn(
                'anthropic package not installed — LLM planning disabled. '
                'Install with: pip install anthropic'
            )

        self._model = model
        self._motion = MotionExecutor(self)
        self._lock = threading.Lock()
        self._busy = False

        self.create_subscription(String, '/vla_instruction', self._instruction_cb, 10)
        self._feedback_pub = self.create_publisher(String, '/vla/task_feedback', 10)

        # Lazy import: ur_interfaces may not be built yet in some envs
        try:
            from ur_interfaces.srv import ExecuteCommand
            self.create_service(ExecuteCommand, '/ur/execute_command', self._service_cb)
            self.get_logger().info('Service /ur/execute_command ready')
        except ImportError:
            self.get_logger().warn(
                'ur_interfaces not found — /ur/execute_command service disabled'
            )

        self.get_logger().info(
            f'LLM planner ready  model={self._model}  '
            f'llm={"on" if self._claude else "off"}'
        )

    # ── Incoming instruction (from handoff coordinator) ───────────────────────

    def _instruction_cb(self, msg: String):
        if self._busy:
            self.get_logger().warn('Already executing — dropping instruction')
            return
        threading.Thread(
            target=self._run,
            args=(msg.data,),
            daemon=True,
        ).start()

    # ── Service (direct / test use) ───────────────────────────────────────────

    def _service_cb(self, request, response):
        ok, msg = self._execute(request.command)
        response.success = ok
        response.message = msg
        return response

    # ── Core execution ────────────────────────────────────────────────────────

    def _run(self, command: str):
        ok, msg = self._execute(command)
        fb = String()
        fb.data = json.dumps({'status': 'completed' if ok else 'failed', 'message': msg})
        self._feedback_pub.publish(fb)

    def _execute(self, command: str) -> tuple[bool, str]:
        with self._lock:
            self._busy = True
            try:
                plan = self._plan(command)
                if plan is None:
                    return False, 'Planning failed'

                self.get_logger().info(f'Plan ({len(plan)} steps): {json.dumps(plan)}')

                if not self._motion.wait_for_servers():
                    return False, 'Motion servers not ready'

                for step in plan:
                    if not self._run_step(step):
                        return False, f'Step failed: {step}'

                return True, 'Done'
            except Exception as exc:
                self.get_logger().error(f'Execution error: {exc}')
                return False, str(exc)
            finally:
                self._busy = False

    def _plan(self, command: str):
        if self._claude is None:
            self.get_logger().error('No Claude client — cannot plan')
            return None

        try:
            response = self._claude.messages.create(
                model=self._model,
                max_tokens=1024,
                system=_SYSTEM_PROMPT,
                messages=[{'role': 'user', 'content': command}],
            )
            text = response.content[0].text.strip()
            # Strip optional markdown fences
            if text.startswith('```'):
                text = text.split('\n', 1)[1].rsplit('```', 1)[0].strip()
            return json.loads(text)
        except Exception as exc:
            self.get_logger().error(f'LLM call failed: {exc}')
            return None

    def _run_step(self, step: dict) -> bool:
        action = step.get('action')

        if action == 'move_to_named_pose':
            return self._motion.move_to_named_pose(
                step.get('group', 'arm'), step['name']
            )

        if action == 'move_to_pose':
            pose = PoseStamped()
            pose.header.frame_id = step.get('frame_id', 'base_link')
            pose.pose.position.x = float(step['x'])
            pose.pose.position.y = float(step['y'])
            pose.pose.position.z = float(step['z'])
            pose.pose.orientation.x = float(step.get('qx', 0.0))
            pose.pose.orientation.y = float(step.get('qy', 0.0))
            pose.pose.orientation.z = float(step.get('qz', 0.0))
            pose.pose.orientation.w = float(step.get('qw', 1.0))
            return self._motion.move_to_pose(pose)

        if action == 'open_gripper':
            return self._motion.open_gripper()

        if action == 'close_gripper':
            return self._motion.close_gripper()

        if action == 'half_close_gripper':
            return self._motion.half_close_gripper()

        self.get_logger().error(f'Unknown action: {action!r}')
        return False


def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()
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
