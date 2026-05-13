"""
easy_full_control_compat.py — rmf_adapter 2.1.8 compatibility shim.

Provides the easy_full_control API (introduced in RMF 2.2+) on top of the
2.1.8 FleetUpdateHandle/RobotCommandHandle/RobotUpdateHandle interfaces.
Automatically used when `import rmf_adapter.easy_full_control` fails.
"""

import datetime
import threading

import numpy as np
import yaml

import rmf_adapter
import rmf_adapter.graph as graph
import rmf_adapter.plan as plan
import rmf_adapter.vehicletraits as traits
import rmf_adapter.geometry as geometry


# ── Public data classes ────────────────────────────────────────────────────────

class RobotCallbacks:
    def __init__(self, navigate_fn, stop_fn, execute_action_fn):
        self.navigate = navigate_fn
        self.stop = stop_fn
        self.execute_action = execute_action_fn


class RobotState:
    def __init__(self, map_name, position, battery_soc):
        self.map_name = map_name
        self.position = position      # [x, y, yaw]
        self.battery_soc = battery_soc


class RobotConfiguration:
    def __init__(self, charger_waypoint=''):
        self.charger_waypoint = charger_waypoint


# ── FleetConfiguration ────────────────────────────────────────────────────────

class FleetConfiguration:
    def __init__(self, fleet_name, vehicle_traits, nav_graph, robots, server_uri=None):
        self._fleet_name = fleet_name
        self._traits = vehicle_traits
        self._nav_graph = nav_graph
        self._robots = robots         # {name: RobotConfiguration}
        self.server_uri = server_uri

    @property
    def fleet_name(self):
        return self._fleet_name

    @property
    def known_robots(self):
        return list(self._robots.keys())

    def get_known_robot_configuration(self, name):
        return self._robots.get(name, RobotConfiguration())

    @staticmethod
    def from_config_files(config_path, nav_graph_path):
        with open(config_path) as f:
            cfg = yaml.safe_load(f)

        fleet = cfg['rmf_fleet']
        lin = fleet['limits']['linear']
        ang = fleet['limits']['angular']
        fp  = fleet['profile']['footprint']
        vic = fleet['profile']['vicinity']

        vehicle_traits = traits.VehicleTraits(
            linear=traits.Limits(lin[0], lin[1]),
            angular=traits.Limits(ang[0], ang[1]),
            profile=traits.Profile(
                footprint=geometry.make_final_convex_circle(fp),
                vicinity=geometry.make_final_convex_circle(vic),
            ),
        )
        vehicle_traits.differential.reversible = fleet.get('reversible', True)

        nav_graph = graph.parse_graph(nav_graph_path, vehicle_traits)

        robots = {}
        for robot_name, robot_data in fleet.get('robots', {}).items():
            robots[robot_name] = RobotConfiguration(
                charger_waypoint=robot_data.get('charger', '')
            )

        return FleetConfiguration(fleet['name'], vehicle_traits, nav_graph, robots)


# ── Internal activity/execution objects ───────────────────────────────────────

class _ActivityIdentifier:
    def is_same(self, other):
        return self is other


class _Execution:
    def __init__(self, done_cb):
        self._done_cb = done_cb
        self._fired = False
        self.identifier = _ActivityIdentifier()

    def finished(self):
        if not self._fired:
            self._fired = True
            self._done_cb()

    def override_schedule(self, map_name, positions, lookahead=None):
        return None


class _Destination:
    def __init__(self, position, map_name, speed_limit=None, dock=None):
        self.position = position      # [x, y, yaw]
        self.map = map_name
        self.speed_limit = speed_limit
        self.dock = dock


# ── RobotCommandHandle bridge ──────────────────────────────────────────────────

class _CommandHandleAdapter(rmf_adapter.RobotCommandHandle):
    """Translates 2.1.8 RobotCommandHandle calls to easy_full_control callbacks."""

    def __init__(self, callbacks: RobotCallbacks, nav_graph):
        super().__init__()
        self._cb = callbacks
        self._nav_graph = nav_graph
        self._lock = threading.Lock()
        self._execution = None

    def follow_new_path(self, waypoints, on_progress, path_finished_callback):
        with self._lock:
            if self._execution:
                self._execution._fired = True  # cancel old silently
            exec_ = _Execution(path_finished_callback)
            self._execution = exec_

        if not waypoints:
            path_finished_callback()
            return

        # Use the last waypoint as the goal destination
        wp = waypoints[-1]
        pos = list(wp.position)          # [x, y, yaw]
        map_name = ''
        try:
            gi = wp.graph_index
            if gi is not None:
                map_name = self._nav_graph.get_waypoint(gi).map_name
        except Exception:
            pass

        dest = _Destination(pos, map_name)
        self._cb.navigate(dest, exec_)

    def stop(self):
        with self._lock:
            exec_ = self._execution
        if exec_:
            self._cb.stop(exec_.identifier)

    def dock(self, dock_name, docking_finished_callback):
        exec_ = _Execution(docking_finished_callback)
        with self._lock:
            self._execution = exec_
        self._cb.execute_action('dock', {'dock_name': dock_name}, exec_)


# ── EasyRobotUpdateHandle ──────────────────────────────────────────────────────

class _EasyRobotUpdateHandle:
    """Wraps 2.1.8 RobotUpdateHandle with the easy_full_control update interface."""

    def __init__(self, handle):
        self._handle = handle

    def update(self, state: RobotState, activity_identifier):
        if state.position and len(state.position) >= 3:
            self._handle.update_position(state.map_name, state.position)
        self._handle.update_battery_soc(state.battery_soc)


# ── _MoreHandle ────────────────────────────────────────────────────────────────

class _MoreHandle:
    def __init__(self, fleet_update_handle, fleet_name):
        self._handle = fleet_update_handle
        self._fleet_name = fleet_name

    @property
    def fleet_name(self):
        return self._fleet_name

    def set_planner_cache_reset_size(self, n):
        pass

    def reassign_dispatched_tasks(self):
        pass

    def open_lanes(self, lanes):
        self._handle.open_lanes(lanes)

    def close_lanes(self, lanes):
        self._handle.close_lanes(lanes)

    def limit_lane_speeds(self, requests):
        pass

    def remove_speed_limits(self, limits):
        pass


# ── EasyFleetHandle ────────────────────────────────────────────────────────────

class _EasyFleetHandle:
    def __init__(self, fleet_update_handle, fleet_config: FleetConfiguration):
        self._handle = fleet_update_handle
        self._config = fleet_config
        self._more = _MoreHandle(fleet_update_handle, fleet_config.fleet_name)

    def more(self):
        return self._more

    def add_robot(self, name, state: RobotState, configuration: RobotConfiguration,
                  callbacks: RobotCallbacks):
        result = [None]
        ready  = threading.Event()

        cmd_handle = _CommandHandleAdapter(callbacks, self._config._nav_graph)

        # Compute starting waypoints
        starts = []
        if state.position and len(state.position) >= 2:
            pos_arr = np.array([
                [state.position[0]],
                [state.position[1]],
                [state.position[2] if len(state.position) > 2 else 0.0],
            ])
            try:
                starts = plan.compute_plan_starts(
                    self._config._nav_graph,
                    state.map_name,
                    pos_arr,
                    datetime.datetime.now(),
                )
            except Exception:
                starts = []

        def handle_cb(update_handle: rmf_adapter.RobotUpdateHandle):
            update_handle.update_battery_soc(state.battery_soc)
            charger = getattr(configuration, 'charger_waypoint', '')
            if charger:
                try:
                    wp = self._config._nav_graph.find_waypoint(charger)
                    if wp:
                        update_handle.set_charger_waypoint(wp.index)
                except Exception:
                    pass
            result[0] = _EasyRobotUpdateHandle(update_handle)
            ready.set()

        self._handle.add_robot(
            cmd_handle,
            name,
            self._config._traits.profile,
            starts,
            handle_cb,
        )

        ready.wait(timeout=10.0)
        return result[0]


# ── Monkey-patch Adapter ───────────────────────────────────────────────────────

def _add_easy_fleet(self, fleet_config: FleetConfiguration):
    fleet_update_handle = self.add_fleet(
        fleet_config.fleet_name,
        fleet_config._traits,
        fleet_config._nav_graph,
        fleet_config.server_uri,
    )
    return _EasyFleetHandle(fleet_update_handle, fleet_config)


rmf_adapter.Adapter.add_easy_fleet = _add_easy_fleet
