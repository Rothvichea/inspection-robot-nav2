# inspection-robot-nav2

> **4-Wheel Steering Autonomous Inspection Robot with Nav2 MPPI Navigation**  
> ROS 2 Humble · Gazebo Classic · SLAM Toolbox · EKF Sensor Fusion · MPPI Controller · Foxglove / RViz

---

## What This Project Does

This is a fully autonomous 4-wheel-steering (4WS) omnidirectional robot that navigates industrial environments using LiDAR-based SLAM, EKF sensor fusion, and the Nav2 MPPI controller. The robot can drive forward, backward, sideways (crab mode), and arc-steer through complex factory layouts while avoiding static and dynamic obstacles in real-time.

```
                         ┌─────────────────────────────────┐
                         │         GOAL POSE               │
                         │   (RViz / Foxglove / API)       │
                         └────────────┬────────────────────┘
                                      │
                                      ▼
┌──────────────────────────────────────────────────────────────────────┐
│                        NAV2 BEHAVIOR TREE                           │
│                  navigate_w_fast_replan.xml                          │
│          Orchestrates: plan → follow → recover → replan              │
└────────┬─────────────────────────────────────┬───────────────────────┘
         │                                     │
         ▼                                     ▼
┌─────────────────────┐             ┌──────────────────────────┐
│   GLOBAL PLANNER    │             │    LOCAL CONTROLLER      │
│   SmacPlanner2D     │──── /plan ──│    MPPI (2000 samples)   │
│                     │             │    40 steps x 0.067s     │
│  A*-based search    │             │    7 critic functions     │
│  on global costmap  │             │    Omni motion model     │
└─────────┬───────────┘             └──────────┬───────────────┘
          │                                    │
          ▼                                    ▼
┌─────────────────────┐             ┌──────────────────────────┐
│   GLOBAL COSTMAP    │             │   VELOCITY SMOOTHER      │
│  Static + Obstacle  │             │   max accel: 1.5 m/s2    │
│  + Inflation layers │             │   max vel: 2.0 m/s       │
│  inflation: 0.65m   │             │   full reverse: -2.0 m/s │
└─────────────────────┘             └──────────┬───────────────┘
                                               │
                                               ▼
                                    ┌──────────────────────┐
                                    │      /cmd_vel        │
                                    └──────────┬───────────┘
                                               │
         ┌─────────────────────────────────────┼────────────────────────┐
         │                                     │                        │
         ▼                                     ▼                        ▼
┌─────────────────┐                 ┌────────────────────┐   ┌──────────────────┐
│    STEERING     │                 │   GAZEBO PLANAR    │   │   STEERING       │
│    ANIMATOR     │                 │   MOVE PLUGIN      │   │   ANIMATOR       │
│  cmd_vel to 4WS │                 │   Body velocity    │   │  Visual wheel    │
│  joint angles   │                 │   execution        │   │  animation       │
└─────────────────┘                 └────────┬───────────┘   └──────────────────┘
                                             │
                                    ┌────────┴───────────┐
                                    │                    │
                                    ▼                    ▼
                         ┌──────────────────┐  ┌─────────────────┐
                         │   WHEEL ODOM     │  │      IMU        │
                         │   /odom          │  │     /imu        │
                         │  x,y,yaw,vx,vy   │  │  gyro + accel   │
                         └────────┬─────────┘  └────────┬────────┘
                                  │                     │
                                  └──────────┬──────────┘
                                             │
                                             ▼
                                  ┌──────────────────────┐
                                  │    EKF FILTER        │
                                  │  robot_localization   │
                                  │  30 Hz fusion         │
                                  │  /odom/filtered       │
                                  │  odom to base_footprint│
                                  └──────────┬───────────┘
                                             │
                                             ▼
                                  ┌──────────────────────┐
                                  │       AMCL           │
                                  │  Particle filter     │
                                  │  2000 particles      │
                                  │  map to odom TF      │
                                  │  Omni motion model   │
                                  └──────────┬───────────┘
                                             │
                                             ▼
                                  ┌──────────────────────┐
                                  │     LiDAR /scan      │
                                  │  + Static Map        │
                                  │  factory_map.yaml    │
                                  └──────────────────────┘
```

### TF Tree

```
map --(AMCL)--> odom --(EKF)--> base_footprint --(URDF)--> base_link
                                                               |-> lidar_link
                                                               |-> imu_link
                                                               |-> front_camera_link
                                                               |-> steering_fl_joint -> wheel_fl
                                                               |-> steering_fr_joint -> wheel_fr
                                                               |-> steering_rl_joint -> wheel_rl
                                                               |-> steering_rr_joint -> wheel_rr
```

### Closed-Loop Control Flow

```
                    ┌──────────────────────────────────────────┐
                    │                                          │
                    ▼                                          │
Goal Pose -> Planner -> MPPI Controller -> Vel Smoother -> /cmd_vel
                                                               │
                                                               ▼
                                                         4WS Robot
                                                               │
                                                               ▼
                                                     Wheel Odom + IMU
                                                               │
                                                               ▼
                                                      EKF -> /odom/filtered
                                                               │
                                                               ▼
                                                    AMCL -> map to odom TF
                                                               │
                                                               └--> Back to Planner + Controller
```

---

## Key Features

| Feature | Description |
|---|---|
| **4WS Omnidirectional** | Forward, backward, crab (sideways), arc steering, spin-in-place |
| **Full Reverse Driving** | `vx_min: -2.0 m/s` with bidirectional MPPI path tracking |
| **MPPI Controller** | 2000 trajectory samples, 7 critics, omni motion model |
| **SLAM Mapping** | slam_toolbox with Ceres solver, loop closure, 5cm resolution |
| **EKF Sensor Fusion** | Wheel odometry + IMU fused at 30 Hz via robot_localization |
| **AMCL Localization** | 2000-particle filter with omni motion model on saved maps |
| **Dynamic Obstacle Avoidance** | Real-time LiDAR obstacle detection + costmap replanning |
| **Footprint-Aware Planning** | Inflation tuned to robot circumscribed radius (0.46m) |
| **Custom Teleop** | Hold-to-move keyboard control with velocity ramping |
| **Recovery Behaviors** | Spin, backup, wait via behavior tree orchestration |
| **Foxglove Compatible** | Browser-based visualization via foxglove_bridge WebSocket |
| **Remote Operation** | Tailscale VPN + WebRTC for long-distance control |

---

## MPPI Critic Weights

| Critic | Weight | Purpose |
|---|---|---|
| GoalCritic | 12.0 | Drive toward goal position |
| PathFollowCritic | 10.0 | Stay close to global planned path |
| PathAlignCritic | 8.0 | Align heading with path direction |
| TwirlingCritic | 8.0 | Penalize unnecessary spinning |
| ObstaclesCritic | 5.0 | Avoid collisions (linear, footprint-aware) |
| ConstraintCritic | 4.0 | Respect velocity and kinematic limits |
| GoalAngleCritic | 3.0 | Face goal only when within 0.8m |
| PathAngleCritic | 1.0 | Allow full 360 degree path following |

---

## Project Structure

```
inspection_robot_ws/
├── src/
│   └── inspection_robot/
│       ├── behavior_trees/
│       │   └── navigate_w_fast_replan.xml    # Nav2 BT with aggressive replanning
│       ├── config/
│       │   ├── ekf.yaml                      # EKF: odom + IMU fusion, 30 Hz
│       │   ├── nav2_params.yaml              # Nav2: MPPI, costmaps, planner, smoother
│       │   └── slam_toolbox_params.yaml      # SLAM: Ceres solver, loop closure
│       ├── launch/
│       │   ├── gazebo_launch.py              # Full nav stack: Gazebo + Nav2 + AMCL + EKF
│       │   └── slam_launch.py                # SLAM mapping mode
│       ├── maps/
│       │   ├── factory_map.pgm               # Occupancy grid (SLAM output)
│       │   └── factory_map.yaml              # Map metadata (origin, resolution)
│       ├── rviz/
│       │   ├── rviz.rviz                     # Navigation visualization config
│       │   └── rviz_slam.rviz                # SLAM mapping visualization config
│       ├── scripts/
│       │   ├── custom_teleop.py              # 4WS keyboard teleop with velocity ramp
│       │   ├── initial_pose_publisher.py     # Auto-publish AMCL initial pose at spawn
│       │   ├── nav2_activator.py             # Lifecycle manager for Nav2 nodes
│       │   ├── odom_tf_publisher.py          # Odom to base_footprint TF broadcaster
│       │   └── steering_animator.py          # cmd_vel to 4WS joint angles (visual)
│       ├── urdf/
│       │   └── inspection_robot.urdf         # Robot description (4WS, LiDAR, IMU, camera)
│       └── worlds/
│           └── factory_world.world           # Gazebo factory environment
├── .gitignore
└── README.md
```

---

## Dependencies

```
ROS 2 Humble
Gazebo Classic (gazebo_ros)
Nav2 (nav2_bringup, nav2_mppi_controller, nav2_smac_planner)
slam_toolbox
robot_localization
foxglove_bridge (optional)
```

---

## Quick Start

### 1. Clone and Build

```bash
git clone https://github.com/Rothvichea/inspection-robot-nav2.git
cd inspection-robot-nav2

# Install ROS 2 dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Navigation (pre-built map)

```bash
# Terminal 1: Full navigation stack
ros2 launch inspection_robot gazebo_launch.py

# Terminal 2: Teleop control (optional)
ros2 run inspection_robot custom_teleop.py
```

Send goals via RViz "2D Goal Pose" or Foxglove Studio.

### 3. Launch SLAM (map a new environment)

```bash
# Terminal 1: SLAM mode
ros2 launch inspection_robot slam_launch.py

# Terminal 2: Drive around with teleop
ros2 run inspection_robot custom_teleop.py

# Terminal 3: Save map when done
ros2 run nav2_map_server map_saver_cli -f ~/inspection_robot_ws/src/inspection_robot/maps/factory_map
```

### 4. Foxglove Visualization (optional, replaces RViz)

```bash
# Install bridge
sudo apt install ros-humble-foxglove-bridge

# Run alongside your stack
ros2 run foxglove_bridge foxglove_bridge --ros-args -p use_sim_time:=true -p port:=8765

# Open Foxglove Studio, connect to ws://localhost:8765
```

---

## Teleop Controls

```
╔═══════════════════════════════════════════════════════╗
║        INSPECTION ROBOT   4WS TELEOP CONTROL         ║
╠═══════════════════════════════════════════════════════╣
║  CAR STEERING                                         ║
║    w = Forward           s = Backward                 ║
║    a = Steer Left        d = Steer Right              ║
║                                                       ║
║  DRIVE + STEER ARCS                                   ║
║    q = Fwd + Left        e = Fwd + Right              ║
║    z = Bwd + Left        c = Bwd + Right              ║
║                                                       ║
║  CRAB MODE  (all 4 wheels turn sideways)              ║
║    j = Crab Left         l = Crab Right               ║
║    u = Fwd + Crab Left   o = Fwd + Crab Right         ║
║                                                       ║
║  h     = Home wheels     SPACE = Emergency Stop       ║
║  Hold key to move, release to stop                    ║
╚═══════════════════════════════════════════════════════╝
```

---

## Configuration Summary

### Costmaps

| Parameter | Global | Local |
|---|---|---|
| Update frequency | 10 Hz | 15 Hz |
| Resolution | 0.05 m/cell | 0.05 m/cell |
| Inflation radius | 0.65 m | 0.65 m |
| Cost scaling factor | 6.0 | 6.0 |
| Obstacle max range | 14.0 m | 9.0 m |
| Footprint | 0.76 x 0.52 m | 0.76 x 0.52 m |

### MPPI Controller

| Parameter | Value |
|---|---|
| Batch size | 2000 trajectories |
| Time steps | 40 (2.68s horizon) |
| Model dt | 0.067 s |
| Temperature | 0.15 |
| Max forward vel | 2.0 m/s |
| Max reverse vel | -2.0 m/s |
| Max lateral vel | 2.0 m/s |
| Max angular vel | 1.8 rad/s |
| Motion model | Omni |

### EKF Sensor Fusion

| Source | Topic | Provides |
|---|---|---|
| Wheel odometry | /odom | x, y, yaw, vx, vy, vyaw |
| IMU | /imu | vyaw (gyro), ax, ay (accel) |
| **Output** | **/odom/filtered** | **Fused state at 30 Hz** |

---

## Robot Specifications

```
  ┌──────────────────────────────┐
  │        0.76 m (length)       │
  │  ┌──┐                ┌──┐   │
  │  │FL│   < LiDAR >    │FR│   │  0.52 m
  │  └──┘    [Camera]     └──┘   │  (width)
  │          [IMU]               │
  │  ┌──┐                ┌──┐   │
  │  │RL│                │RR│   │
  │  └──┘                └──┘   │
  └──────────────────────────────┘
  
  4 steering joints (each wheel pivots independently)
  Circumscribed radius: 0.46 m
  Wheelbase: 0.50 m
  Track width: 0.53 m
```

---

## Video Demos

https://www.youtube.com/watch?v=YOr-bXfJsyM

---

## Author

**Rothvichea CHEA** - Robotics / Mechatronics Engineer  
[Portfolio](https://rothvicheachea.netlify.app) | [LinkedIn](https://linkedin.com/in/chea-rothvichea-a96154227) | [GitHub](https://github.com/Rothvichea)
