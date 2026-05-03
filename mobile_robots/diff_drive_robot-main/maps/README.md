Generated maps are saved here (map_*.yaml / map_*.pgm).

Naming convention: map_<world_name>.yaml  (e.g. map_maze.yaml, map_obstacles.yaml)

Multi-robot launch

Launch Gazebo with the GUI:
  source /opt/ros/humble/setup.bash
  source ~/rosnav/install/setup.bash
  ros2 launch diff_drive_robot multi_robot.launch.py

Launch headless:
  source /opt/ros/humble/setup.bash
  source ~/rosnav/install/setup.bash
  ros2 launch diff_drive_robot multi_robot.launch.py headless:=true rviz:=false

Before relaunching, stop any older stack first to avoid duplicate /clock and TF publishers:
  pkill -f "ros2 launch diff_drive_robot multi_robot.launch.py"
  pkill -f "gz sim"

Notes

- `robot1` runs SLAM and builds the shared `/map`.
- `robot2` localizes on that shared map with AMCL.
- Current launch expects a global `/tf` tree for both robots.

Map generation

To generate a fresh map for any world:
  ros2 launch diff_drive_robot multi_robot.launch.py world:=maze

To manually save the current SLAM map:
  ros2 run nav2_map_server map_saver_cli -f src/diff_drive_robot-main/maps/map_maze

To use fleet_manager to save:
  ros2 run diff_drive_robot fleet_manager.py savemap src/diff_drive_robot-main/maps/map_maze
