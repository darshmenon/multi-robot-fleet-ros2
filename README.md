# multi-robot-fleet-ros2

A ROS 2 monorepo for multi-robot fleet management, combining autonomous mobile robots (AMRs), mobile manipulators (UR3), and fleet coordination via Open-RMF.

---

## Repository Structure

```
multi-robot-fleet-ros2/
├── mobile_robots/               # AMR navigation and mobile pick-place robots
│   ├── diff_drive_robot-main/   # AMR (Differential Drive) with Nav2 and SLAM
│   └── pickplace_rl_mobile/     # Mobile Manipulator with RL-based pick & place
│
├── manipulation/                # Arm descriptions, MoveIt config, and demos
│   ├── ur_description/          # UR3 URDF/xacro robot description
│   ├── onrobot_description/     # OnRobot gripper description
│   ├── robotiq_description/     # Robotiq gripper description
│   ├── robotiq_2f_85_gripper_visualization/
│   ├── moveit_config/           # MoveIt 2 configuration for UR3
│   └── ur_moveit_demos/         # MoveIt planning and execution demos
│
├── fleet_management/            # Multi-robot fleet coordination
│   └── rmf_demos/               # Open-RMF demos and fleet adapter
│
├── simulation/                  # Gazebo simulation environments
│   └── ur_gazebo/               # Warehouse and pick & place worlds (Gazebo Harmonic/GZ)
│
├── ai_skills/                   # AI-powered robot capabilities
│   └── ur_data_collector/       # Data collection for robot learning
│
├── interfaces/                  # Shared ROS 2 message/service interfaces
│   └── ur_interfaces/
│
├── tests/                       # Integration and system tests
│   └── testing/
│
└── docs/                        # Architecture docs, guides, and notes
```

---

## Packages Overview

### Mobile Robots & AMRs
| Package | Description |
|---|---|
| `diff_drive_robot-main` | Autonomous Mobile Robot (AMR) platform with Nav2, SLAM Toolbox, and multi-robot coordination support. |
| `pickplace_rl_mobile` | Mobile Manipulator platform using Reinforcement Learning policies for intelligent warehouse pick & place. |

### Manipulation
| Package | Description |
|---|---|
| `ur_description` | UR3 robot URDF and xacro description |
| `moveit_config` | MoveIt 2 config — planning pipelines, kinematics, controllers |
| `ur_moveit_demos` | Demo nodes for planning, execution, and custom motions |
| `robotiq_description` | Robotiq 2F-85 gripper URDF |
| `onrobot_description` | OnRobot gripper description |

### Fleet Management
| Package | Description |
|---|---|
| `rmf_demos` | Open-RMF fleet adapter, task dispatcher, and map demos for heterogeneous fleet coordination. |

### Simulation
| Package | Description |
|---|---|
| `ur_gazebo` | Gazebo worlds and simulation setup. Optimized for Gazebo Harmonic (GZ). Includes warehouse and pick & place environments. |

### AI Skills
| Package | Description |
|---|---|
| `ur_data_collector` | ROS 2 node for collecting training data from robot demos |
| `ur_llm_planner` | Natural-language motion planner — converts free-text commands to UR arm action sequences via a local Ollama model (default) or Anthropic Claude. Wired into the handoff coordinator FSM via `/vla_instruction` / `/vla/task_feedback`. |

---

## Getting Started

### Prerequisites
- ROS 2 Humble
- MoveIt 2
- Nav2
- Gazebo Harmonic (GZ)
- Open-RMF Core

### Open-RMF Setup (Avoiding Gazebo Conflicts)
If you are using Gazebo Harmonic/GZ, avoid installing the full `ros-humble-rmf-dev` package as it may conflict with Gazebo Classic. Instead, install the core components:

```bash
sudo apt install -y \
  ros-humble-rmf-traffic \
  ros-humble-rmf-traffic-ros2 \
  ros-humble-rmf-fleet-adapter \
  ros-humble-rmf-fleet-adapter-python \
  ros-humble-rmf-fleet-msgs \
  ros-humble-rmf-task \
  ros-humble-rmf-task-ros2 \
  ros-humble-rmf-task-msgs \
  ros-humble-rmf-task-sequence \
  ros-humble-rmf-building-map-msgs \
  ros-humble-rmf-utils \
  ros-humble-rmf-api-msgs \
  ros-humble-rmf-websocket
```

### Build
```bash
cd multi-robot-fleet-ros2
colcon build --symlink-install
source install/setup.bash
```

### Launch AMR (Nav2 + SLAM)
```bash
ros2 launch diff_drive_robot-main bringup.launch.py
```

### Launch UR3 with MoveIt
```bash
ros2 launch moveit_config ur3_moveit.launch.py
```

### Launch Fleet Demo (Open-RMF)
```bash
ros2 launch rmf_demos office.launch.xml
```

### Launch LLM Planner (Ollama)
Install and start Ollama, then pull a model:
```bash
ollama pull llama2   # or mistral, llama3.2, etc.
```

Launch the planner node (defaults to `llama2` on `http://localhost:11434`):
```bash
ros2 launch ur_llm_planner llm_planner.launch.py

# Different model
ros2 launch ur_llm_planner llm_planner.launch.py model:=mistral

# Anthropic Claude fallback
ros2 launch ur_llm_planner llm_planner.launch.py backend:=anthropic model:=claude-haiku-4-5-20251001
```

Send a natural language command directly:
```bash
ros2 service call /ur/execute_command ur_interfaces/srv/ExecuteCommand \
  '{command: "pick the red box from the AMR and place it on the shelf"}'
```

Or trigger via the handoff coordinator (the planner subscribes to `/vla_instruction` automatically):
```bash
ros2 topic pub --once /vla_instruction std_msgs/msg/String \
  '{data: "pick the box_A from the AMR and place it on the shelf"}'
```

### Evaluate planner accuracy
Run the offline accuracy evaluator against a live Ollama instance (no ROS 2 needed):
```bash
python3 tests/eval_planner_accuracy.py

# Different model or URL
python3 tests/eval_planner_accuracy.py --model mistral
python3 tests/eval_planner_accuracy.py --url http://192.168.1.10:11434 --model llama3.2

# Show full plan output for every test case
python3 tests/eval_planner_accuracy.py --verbose
```

Checks each plan for: valid JSON, known actions only, correct gripper ordering (open before close), starts with a named pose, ends at home, and valid Cartesian field types.
Exit code 0 if ≥ 60% of checks pass, 1 otherwise — suitable for CI.

---

## Roadmap
- [x] Multi-robot Nav2 navigation with SLAM
- [x] UR3 MoveIt 2 integration
- [x] Open-RMF fleet coordination demos
- [x] LLM motion planner (Ollama + Anthropic backends)
- [ ] RMF fleet adapter for diff_drive_robot AMRs
- [ ] RMF fleet adapter for UR3 mobile manipulator
- [ ] Centralized heterogeneous fleet dispatcher (AMRs + arms)
- [ ] RMF traffic editor map for the Gazebo warehouse world
- [x] Inter-robot handoff — AMR delivers object to UR3 pick zone
- [ ] Sim-to-real transfer for RL pick & place policies
- [ ] Object detection node wired to DetectedObject msgs
- [ ] Multi-robot map merging (SLAM Toolbox multirobot mode)
- [ ] Docker / devcontainer for reproducible builds

---

## What We Can Build Next

### RMF Integration
| Feature | Description |
|---|---|
| **Custom fleet adapter** | Write a `rmf_fleet_adapter`-compliant node for the `diff_drive_robot` so RMF can dispatch and monitor Nav2 tasks directly. |
| **Traffic editor map** | Create a `.building.yaml` map in the RMF Traffic Editor matching the Gazebo warehouse, enabling lift/door/charger integration. |
| **Patrol & delivery tasks** | Configure RMF `patrol` and `delivery` task types dispatched through the RMF web dashboard or API. |
| **RMF web dashboard** | Launch the Open-RMF web UI (`rmf-web`) to visualize robot states, task queues, and traffic lanes in real time. |
| **Mobile manipulator adapter** | Extend RMF to treat the `pickplace_rl_mobile` robot as a `robot_type` that can accept pick-and-place task payloads. |
| **Heterogeneous task dispatch** | Use RMF's task bidding system to route delivery tasks to AMRs and manipulation tasks to UR3 robots based on capability. |

### Multi-Robot Capabilities
| Feature | Description |
|---|---|
| **Map merging** | Use SLAM Toolbox's multirobot mode (already has config) to merge maps from multiple AMRs into a single global costmap. |
| **Inter-robot handoff** | AMR navigates to a handoff zone → UR3 arm picks the payload → AMR continues delivery. Coordinate via shared ROS 2 topics or RMF tasks. |
| **Centralized mission server** | Extend the existing `mission_server.py` to accept high-level goals (e.g. "deliver box from A to B") and decompose them into Nav2 + MoveIt subtasks. |
| **Fleet health dashboard** | Extend `fleet_health.py` and `fleet_gui.py` with battery, pose, and task-status telemetry for all robots in a single Tkinter or web view. |
| **Dynamic task reallocation** | If a robot fails mid-task, the task allocator re-bids and assigns to another available robot. |
| **Priority-based traffic management** | Assign lane priority so high-priority robots (e.g. emergency delivery) preempt lower-priority AMRs at intersections. |

### AI / Learning
| Feature | Description |
|---|---|
| **Swap Ollama model** | Try `mistral` or `llama3.2` by launching with `model:=mistral` — no code changes needed. |
| **Object detection node** | Add a YOLO or DepthAI node publishing `DetectedObjectArray`; feed detected poses directly into the LLM planner prompt. |
| **Rosbag training loop** | Automate: record demo → replay → extract episodes → run `train_bc.py` behavior cloning in one launch file. |
| **Restore SmolVLA inference** | Rebuild the deleted `ur_smolvla` package with an updated model checkpoint wired as a second LLM planner backend. |

---

## Author
**darshmenon** — [github.com/darshmenon](https://github.com/darshmenon)
