"""Swap the Nav2 local controller in a params file without keeping a YAML copy per controller.

`apply_controller(src, name)` returns the path of a generated params file whose
`controller_server.FollowPath` block is replaced by one of the presets below.
`mppi` (the default) returns the source file unchanged.
"""
import copy
import hashlib
import os
import tempfile

import yaml

_RPP = {
    'plugin': 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
    'desired_linear_vel': 0.5,
    'lookahead_dist': 0.6,
    'min_lookahead_dist': 0.3,
    'max_lookahead_dist': 0.9,
    'lookahead_time': 1.5,
    'rotate_to_heading_angular_vel': 1.8,
    'transform_tolerance': 0.1,
    'use_velocity_scaled_lookahead_dist': True,
    'min_approach_linear_velocity': 0.05,
    'approach_velocity_scaling_dist': 0.6,
    'use_collision_detection': True,
    'max_allowed_time_to_collision_up_to_carrot': 1.0,
    'use_regulated_linear_velocity_scaling': True,
    'regulated_linear_scaling_min_radius': 0.9,
    'regulated_linear_scaling_min_speed': 0.25,
    'use_cost_regulated_linear_velocity_scaling': False,
    'use_rotate_to_heading': True,
    'allow_reversing': False,
    'rotate_to_heading_min_angle': 0.785,
    'max_angular_accel': 3.2,
}

_GRACEFUL = {
    'plugin': 'nav2_graceful_controller::GracefulController',
    'transform_tolerance': 0.1,
    'motion_target_dist': 0.6,
    'initial_rotation': True,
    'initial_rotation_tolerance': 0.75,
    'prefer_final_rotation': True,
    'rotation_scaling_factor': 0.5,
    'allow_backward': False,
    'k_phi': 2.0,
    'k_delta': 1.0,
    'beta': 0.4,
    'lambda': 2.0,
    'v_linear_min': 0.1,
    'v_linear_max': 0.6,
    'v_angular_max': 1.5,
    'slowdown_radius': 1.0,
}

_SHIM = {
    'plugin': 'nav2_rotation_shim_controller::RotationShimController',
    'primary_controller': _RPP['plugin'],
    'angular_dist_threshold': 0.785,
    'forward_sampling_distance': 0.5,
    'rotate_to_heading_angular_vel': 1.8,
    'max_angular_accel': 3.2,
    'simulate_ahead_time': 1.0,
}

PRESETS = {'rpp': _RPP, 'graceful': _GRACEFUL, 'shim': {**_SHIM, **{k: v for k, v in _RPP.items() if k != 'plugin'}}}
CHOICES = ['mppi'] + sorted(PRESETS)


def apply_controller(src: str, name: str, out_dir: str = '') -> str:
    name = (name or 'mppi').strip().lower()
    if name == 'mppi':
        return src
    if name not in PRESETS:
        raise ValueError(f'unknown controller {name!r}, choose from {CHOICES}')
    with open(src) as f:
        data = yaml.safe_load(f)
    params = data['controller_server']['ros__parameters']
    params['FollowPath'] = copy.deepcopy(PRESETS[name])
    tag = hashlib.md5(f'{src}:{name}'.encode()).hexdigest()[:8]
    out_dir = out_dir or tempfile.gettempdir()
    out = os.path.join(out_dir, f'nav2_params_{name}_{tag}.yaml')
    with open(out, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    return out
