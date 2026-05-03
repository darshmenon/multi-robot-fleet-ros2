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
│   ├── ur_smolvla/              # SmolVLA vision-language-action inference
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
| `ur_smolvla` | SmolVLA inference node for vision-language-action control |
| `ur_data_collector` | ROS 2 node for collecting training data from robot demos |

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

---

## Roadmap
- [x] Multi-robot Nav2 navigation with SLAM
- [x] UR3 MoveIt 2 integration
- [x] Open-RMF fleet coordination demos
- [x] SmolVLA vision-language-action inference
- [ ] Centralized heterogeneous fleet dispatcher
- [ ] Sim-to-real transfer for RL pick & place policies
- [ ] RMF fleet adapter for UR3 mobile manipulator

---

## Author
**darshmenon** — [github.com/darshmenon](https://github.com/darshmenon)
