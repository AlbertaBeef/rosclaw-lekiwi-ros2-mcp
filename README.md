# rosclaw-lekiwi-ros2-mcp

ROSClaw MCP Server for **LeKiwi Mobile Base + SO-101 Arm** via ROS2.

Part of the [ROSClaw](https://github.com/ros-claw) Embodied Intelligence Operating System.

## Overview

This MCP server enables LLM agents to control a LeKiwi omnidirectional mobile robot with a mounted SO-101 6-DOF arm through the Model Context Protocol. It combines base movement (3-wheel omni-drive) with arm manipulation.

```
LLM Agent  ──MCP──►  rosclaw-lekiwi-ros2-mcp  ──ROS2──►  lekiwi_motor_bridge  ──Serial──►  LeKiwi + SO-101
```

## Hardware

| Component | Details |
|-----------|---------|
| Base | LeKiwi 3-wheel omnidirectional (motor IDs 7, 8, 9) |
| Arm | SO-101 6-DOF (motor IDs 1-6) |
| Servos | Feetech STS3215 x9 |
| Protocol | ROS2 via lekiwi_motor_bridge |
| Base control | `/cmd_vel` (Twist) |
| Arm control | `/joint_commands` (JointState) |
| State feedback | `/joint_states` (JointState) |

## Installation

```bash
source /opt/ros/humble/setup.bash

# Install SO-101 dependency first
cd ../rosclaw-so101-ros2-mcp && pip install -e .

# Install this package
cd ../rosclaw-lekiwi-ros2-mcp && pip install -e .
```

## Prerequisites

The `lekiwi_motor_bridge` node must be running with SO-101 enabled:

```bash
ros2 run lekiwi_hw_interface lekiwi_motor_bridge --ros-args \
  -p port:=/dev/ttyACM0 \
  -p use_so101:=true
```

## Run as MCP Server

```bash
source /opt/ros/humble/setup.bash
python src/lekiwi_mcp_server.py
```

## Available Tools

### Connection
| Tool | Description |
|------|-------------|
| `connect_lekiwi` | Connect to LeKiwi via ROS2 |
| `disconnect_lekiwi` | Disconnect |

### Base Control
| Tool | Description |
|------|-------------|
| `move_base` | Move base with velocity for a duration, then stop |
| `stop_base` | Stop base immediately |

### Arm Control
| Tool | Description |
|------|-------------|
| `move_arm_to_home` | Move arm to zero position |
| `move_arm_joint` | Move a single joint |
| `move_arm_joints` | Move multiple joints simultaneously |
| `open_gripper` | Open gripper |
| `close_gripper` | Close gripper |
| `stop_arm` | Hold arm at current position |

### Combined
| Tool | Description |
|------|-------------|
| `stop_all` | Stop base and arm |
| `get_robot_state` | Get full state (arm + base) |

## Available Resources

| Resource | Description |
|----------|-------------|
| `lekiwi://status` | Arm positions, gripper, wheel positions |
| `lekiwi://joints` | Arm joint limits + base speed limits |
| `lekiwi://connection` | ROS2 connection status |

## Base Reference

| Parameter | Value |
|-----------|-------|
| Max linear speed | 0.3 m/s |
| Max angular speed | 1.57 rad/s (~90 deg/s) |
| Wheel radius | 0.054 m |
| Base radius | 0.125 m |
| Drive type | 3-wheel omnidirectional |

## Arm Reference (SO-101)

| Joint | Range (rad) | Motor ID |
|-------|-------------|----------|
| shoulder_pan | -2.055 to +2.058 | 1 |
| shoulder_lift | -2.018 to +2.018 | 2 |
| elbow_flex | -1.653 to +1.654 | 3 |
| wrist_flex | -1.786 to +1.790 | 4 |
| wrist_roll | -3.194 to +4.120 | 5 |
| gripper | -0.175 to +1.745 | 6 |

## Architecture

```
lekiwi_mcp_server.py
├── OmniBaseKinematics  — 3-wheel forward/inverse kinematics
├── LeKiwiState          — Combined base + arm state dataclass
├── LeKiwiROS2Bridge     — ROS2 Node (extends rclpy.Node)
│   ├── _joint_state_callback()  — /joint_states (arm + wheels)
│   ├── publish_base_velocity()  — /cmd_vel publisher
│   ├── publish_arm_command()    — /joint_commands publisher
│   ├── validate_base_velocity() — Speed limit check
│   └── validate_arm_positions() — Delegates to SO101ROS2Bridge
└── MCP Tools            — FastMCP tool definitions

Depends on: rosclaw-so101-ros2-mcp (SO101ROS2Bridge, SO101State)
```

## License

MIT License

## Part of ROSClaw

- [rosclaw-so101-ros2-mcp](https://github.com/ros-claw/rosclaw-so101-ros2-mcp) — SO-101 arm standalone
- [rosclaw-ur-ros2-mcp](https://github.com/ros-claw/rosclaw-ur-ros2-mcp) — UR5 arm (ROS2)
- [rosclaw-g1-dds-mcp](https://github.com/ros-claw/rosclaw-g1-dds-mcp) — Unitree G1 (DDS)
- [rosclaw-gimbal-mcp](https://github.com/ros-claw/rosclaw-gimbal-mcp) — GCU Gimbal (Serial)
