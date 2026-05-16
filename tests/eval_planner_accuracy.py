#!/usr/bin/env python3
"""
Accuracy evaluator for the LLM motion planner.

Hits the real Ollama API with a suite of labelled test cases and scores each
plan against structural and semantic rules.  No ROS 2 or robot hardware needed.

Usage:
    python3 tests/eval_planner_accuracy.py
    python3 tests/eval_planner_accuracy.py --model mistral
    python3 tests/eval_planner_accuracy.py --url http://localhost:11434 --model llama2
    python3 tests/eval_planner_accuracy.py --verbose
"""

import argparse
import json
import sys
import textwrap
import time

import requests

# ---------------------------------------------------------------------------
# System prompt (must match llm_planner_node.py)
# ---------------------------------------------------------------------------

SYSTEM_PROMPT = """\
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
- Always start with move_to_named_pose (home or ready) before approaching a target.
- Always open_gripper before reaching the pick pose.
- Return home after placing.
- Use frame_id "base_link" for all Cartesian poses.
- Quaternion (qx=0, qy=0.707, qz=0, qw=0.707) points the tool straight down.
"""

KNOWN_ACTIONS = {
    'move_to_named_pose',
    'move_to_pose',
    'open_gripper',
    'close_gripper',
    'half_close_gripper',
}

NAMED_POSES = {'home', 'ready'}

# ---------------------------------------------------------------------------
# Test suite
# Each case: (command, expected_checks)
# expected_checks: list of check names that MUST pass for this case
# ---------------------------------------------------------------------------

TEST_CASES = [
    {
        'id': 'pick_place_explicit',
        'command': 'pick the box at position x=0.4 y=0.0 z=0.3 and place it on the shelf at x=0.5 y=0.2 z=0.4',
        'checks': ['json_valid', 'known_actions', 'nonempty', 'starts_with_named_pose',
                   'open_before_close', 'has_move_to_pose', 'ends_with_home'],
    },
    {
        'id': 'pick_from_amr',
        'command': 'pick the payload from the AMR and place it on the shelf',
        'checks': ['json_valid', 'known_actions', 'nonempty', 'starts_with_named_pose',
                   'open_before_close', 'has_move_to_pose', 'ends_with_home'],
    },
    {
        'id': 'go_home',
        'command': 'move the arm to home position',
        'checks': ['json_valid', 'known_actions', 'nonempty', 'has_named_pose_home'],
    },
    {
        'id': 'open_gripper',
        'command': 'open the gripper',
        'checks': ['json_valid', 'known_actions', 'nonempty', 'has_open_gripper'],
    },
    {
        'id': 'close_gripper',
        'command': 'close the gripper',
        'checks': ['json_valid', 'known_actions', 'nonempty', 'has_close_gripper'],
    },
    {
        'id': 'ready_pose',
        'command': 'move to the ready position',
        'checks': ['json_valid', 'known_actions', 'nonempty', 'has_named_pose_ready'],
    },
    {
        'id': 'pick_object',
        'command': 'grasp the red box',
        'checks': ['json_valid', 'known_actions', 'nonempty', 'starts_with_named_pose',
                   'open_before_close'],
    },
    {
        'id': 'place_object',
        'command': 'place the object at x=0.3 y=-0.1 z=0.25',
        'checks': ['json_valid', 'known_actions', 'nonempty', 'has_move_to_pose'],
    },
    {
        'id': 'full_pipeline',
        'command': (
            'pick the cylinder from the AMR at position x=0.35 y=0.0 z=0.28 '
            'and put it down on the conveyor at x=0.6 y=0.15 z=0.3'
        ),
        'checks': ['json_valid', 'known_actions', 'nonempty', 'starts_with_named_pose',
                   'open_before_close', 'has_move_to_pose', 'ends_with_home',
                   'pose_fields_valid'],
    },
    {
        'id': 'coordinate_extraction',
        'command': 'move the tool to x=0.5 y=0.1 z=0.4',
        'checks': ['json_valid', 'known_actions', 'nonempty', 'has_move_to_pose',
                   'pose_fields_valid'],
    },
]

# ---------------------------------------------------------------------------
# Check implementations
# ---------------------------------------------------------------------------

def _actions(plan):
    return [s.get('action') for s in plan if isinstance(s, dict)]


def check_json_valid(plan, raw):
    return True, ''  # if we got here, JSON was parsed


def check_nonempty(plan, raw):
    ok = len(plan) > 0
    return ok, '' if ok else 'plan is empty'


def check_known_actions(plan, raw):
    bad = [a for a in _actions(plan) if a not in KNOWN_ACTIONS]
    return (not bad), (f'unknown actions: {bad}' if bad else '')


def check_starts_with_named_pose(plan, raw):
    if not plan:
        return False, 'empty plan'
    first = plan[0].get('action') if isinstance(plan[0], dict) else None
    ok = first == 'move_to_named_pose'
    return ok, '' if ok else f'first action is {first!r}, expected move_to_named_pose'


def check_open_before_close(plan, raw):
    acts = _actions(plan)
    try:
        oi = acts.index('open_gripper')
    except ValueError:
        return False, 'open_gripper not found'
    try:
        ci = next(i for i, a in enumerate(acts) if a in ('close_gripper', 'half_close_gripper'))
    except StopIteration:
        return False, 'close_gripper not found'
    ok = oi < ci
    return ok, '' if ok else f'open_gripper (idx {oi}) after close_gripper (idx {ci})'


def check_has_move_to_pose(plan, raw):
    ok = 'move_to_pose' in _actions(plan)
    return ok, '' if ok else 'no move_to_pose step'


def check_ends_with_home(plan, raw):
    if not plan:
        return False, 'empty plan'
    last = plan[-1] if isinstance(plan[-1], dict) else {}
    ok = last.get('action') == 'move_to_named_pose' and last.get('name') == 'home'
    return ok, '' if ok else f'last step is {last}'


def check_has_named_pose_home(plan, raw):
    ok = any(
        s.get('action') == 'move_to_named_pose' and s.get('name') == 'home'
        for s in plan if isinstance(s, dict)
    )
    return ok, '' if ok else 'no move_to_named_pose home'


def check_has_named_pose_ready(plan, raw):
    ok = any(
        s.get('action') == 'move_to_named_pose' and s.get('name') == 'ready'
        for s in plan if isinstance(s, dict)
    )
    return ok, '' if ok else 'no move_to_named_pose ready'


def check_has_open_gripper(plan, raw):
    ok = 'open_gripper' in _actions(plan)
    return ok, '' if ok else 'no open_gripper'


def check_has_close_gripper(plan, raw):
    ok = any(a in ('close_gripper', 'half_close_gripper') for a in _actions(plan))
    return ok, '' if ok else 'no close/half_close_gripper'


def check_pose_fields_valid(plan, raw):
    errors = []
    for step in plan:
        if not isinstance(step, dict) or step.get('action') != 'move_to_pose':
            continue
        for field in ('x', 'y', 'z'):
            if field not in step:
                errors.append(f'move_to_pose missing {field!r}')
                continue
            try:
                float(step[field])
            except (TypeError, ValueError):
                errors.append(f'move_to_pose {field} not numeric: {step[field]!r}')
        for field in ('qx', 'qy', 'qz', 'qw'):
            if field in step:
                try:
                    float(step[field])
                except (TypeError, ValueError):
                    errors.append(f'move_to_pose {field} not numeric')
    return (not errors), '; '.join(errors)


CHECKERS = {
    'json_valid':            check_json_valid,
    'nonempty':              check_nonempty,
    'known_actions':         check_known_actions,
    'starts_with_named_pose': check_starts_with_named_pose,
    'open_before_close':     check_open_before_close,
    'has_move_to_pose':      check_has_move_to_pose,
    'ends_with_home':        check_ends_with_home,
    'has_named_pose_home':   check_has_named_pose_home,
    'has_named_pose_ready':  check_has_named_pose_ready,
    'has_open_gripper':      check_has_open_gripper,
    'has_close_gripper':     check_has_close_gripper,
    'pose_fields_valid':     check_pose_fields_valid,
}

# ---------------------------------------------------------------------------
# Ollama call
# ---------------------------------------------------------------------------

def call_ollama(command, model, base_url, timeout=120):
    url = f'{base_url.rstrip("/")}/api/chat'
    payload = {
        'model': model,
        'messages': [
            {'role': 'system', 'content': SYSTEM_PROMPT},
            {'role': 'user', 'content': command},
        ],
        'stream': False,
    }
    resp = requests.post(url, json=payload, timeout=timeout)
    resp.raise_for_status()
    return resp.json()['message']['content'].strip()


def parse_plan(text):
    if '```' in text:
        text = text.split('```', 1)[1]
        if text.startswith('json'):
            text = text[4:]
        text = text.rsplit('```', 1)[0].strip()
    start = text.find('[')
    if start != -1:
        text = text[start:]
    return json.loads(text)


# ---------------------------------------------------------------------------
# Evaluator
# ---------------------------------------------------------------------------

def evaluate(model, base_url, verbose, timeout=120, only_cases=None):
    GREEN  = '\033[32m'
    RED    = '\033[31m'
    YELLOW = '\033[33m'
    RESET  = '\033[0m'
    BOLD   = '\033[1m'

    total_checks = passed_checks = 0
    case_results = []

    cases = TEST_CASES
    if only_cases:
        cases = [c for c in TEST_CASES if c['id'] in only_cases]

    print(f'\n{BOLD}LLM Planner Accuracy Evaluation{RESET}')
    print(f'Model : {model}')
    print(f'URL   : {base_url}')
    print(f'Cases : {len(cases)}\n')
    print('-' * 70)

    for case in cases:
        cid = case['id']
        cmd = case['command']
        checks = case['checks']

        # Call Ollama
        raw = None
        plan = None
        parse_err = None
        latency = None

        try:
            t0 = time.monotonic()
            raw = call_ollama(cmd, model, base_url, timeout=timeout)
            latency = time.monotonic() - t0
            plan = parse_plan(raw)
        except json.JSONDecodeError as e:
            parse_err = f'JSON parse error: {e}'
        except Exception as e:
            parse_err = f'API error: {e}'

        # Run checks
        results = {}
        for check_name in checks:
            if check_name == 'json_valid' and parse_err:
                results[check_name] = (False, parse_err)
                continue
            if parse_err:
                results[check_name] = (False, 'skipped (parse failed)')
                continue
            fn = CHECKERS.get(check_name)
            if fn is None:
                results[check_name] = (False, f'unknown checker: {check_name}')
                continue
            results[check_name] = fn(plan, raw)

        case_pass = all(ok for ok, _ in results.values())
        n_pass = sum(1 for ok, _ in results.values() if ok)
        n_total = len(results)
        total_checks += n_total
        passed_checks += n_pass

        status = f'{GREEN}PASS{RESET}' if case_pass else f'{RED}FAIL{RESET}'
        lat_str = f'{latency:.1f}s' if latency else '—'
        print(f'{status}  [{n_pass}/{n_total}]  {cid}  ({lat_str})')

        if verbose or not case_pass:
            print(f'       cmd: {textwrap.shorten(cmd, 70)}')
            for check_name, (ok, msg) in results.items():
                sym = f'{GREEN}✓{RESET}' if ok else f'{RED}✗{RESET}'
                detail = f'  — {msg}' if msg else ''
                print(f'         {sym} {check_name}{detail}')
            if verbose and plan is not None:
                print(f'       plan: {json.dumps(plan, separators=(",", ":"))}')
            print()

        case_results.append((cid, case_pass, n_pass, n_total))

    print('-' * 70)
    pct = 100 * passed_checks // total_checks if total_checks else 0
    cases_passed = sum(1 for _, ok, _, _ in case_results if ok)
    color = GREEN if pct >= 80 else (YELLOW if pct >= 60 else RED)
    print(f'\n{BOLD}Result: {color}{cases_passed}/{len(cases)} cases  '
          f'{passed_checks}/{total_checks} checks  {pct}%{RESET}\n')

    return pct >= 60  # exit 0 if ≥60% checks pass


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Evaluate LLM planner accuracy via Ollama')
    parser.add_argument('--model',   default='llama2',                 help='Ollama model name')
    parser.add_argument('--url',     default='http://localhost:11434',  help='Ollama base URL')
    parser.add_argument('--timeout', type=int, default=120,            help='Per-request timeout in seconds')
    parser.add_argument('--cases',   nargs='+',                        help='Run only these case IDs')
    parser.add_argument('--verbose', action='store_true',              help='Show plan output for every case')
    args = parser.parse_args()

    ok = evaluate(model=args.model, base_url=args.url, verbose=args.verbose,
                  timeout=args.timeout, only_cases=args.cases)
    sys.exit(0 if ok else 1)
