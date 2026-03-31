# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a **ROS 2 quadruped robot VMC (Virtual Model Control) system** for a **12-DOF robot dog** using RobStride motors. The project is in early development - some packages are stubs while the motor control node is functional.

## Build Commands

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select <package_name>

# Source the workspace
source ./install/setup.zsh  # or setup.bash
```

## CAN Bus Setup

Before running motor nodes, configure CAN interfaces:

```bash
# Run once per session (or add to ~/.bashrc)
./setup_can.sh
```

This configures can0-can3 at 1Mbps with txqueuelen 1000.

## Running the Motor Control Node

```bash
# Run the RobStride motor control node
ros2 run rs_motor_ros2 rs_motor_ros2

# IMPORTANT: Binary path issue - ensure using ~/VMC_Quadruped/install/rs_motor_ros2/
# not ~/VMC_Quadruped/src/robstride_ros_sample/install/
```

## Key Topics

| Topic | Type | Direction | Description |
|-------|------|----------|-------------|
| `/motor_command` | `interfaces::msg::MotorCommand12` | subscribe | 12-motor torque/position/velocity/kp/kd commands |
| `/motor_feedback` | `interfaces::msg::MotorFeedback12` | publish | 12-motor torque/position/velocity/temperature at 10ms |

## Package Architecture

- **`interfaces`** - ROS message definitions (BaseState, ContactState, GaitPhase, MotionCommand, MotorCommand12, MotorFeedback12)
- **`robstride_ros_sample`** - Active motor control node (CAN bus communication with RobStride motors)
- **`description`** - Robot URDF/xacro models (stub)
- **`estimation`** - State estimation: base pose/velocity, contact state (stub)
- **`main_control`** - VMC controller implementation (stub)
- **`gait`** - Gait scheduling (planned, not yet created)

## Motor Configuration

12 motors across 4 CAN channels (can0-can3, 3 motors each):
- fl_hip, fl_thigh, fl_calf → can0
- fr_hip, fr_thigh, fr_calf → can1
- rl_hip, rl_thigh, rl_calf → can2
- rr_hip, rr_thigh, rr_calf → can3

Uses ROBSTRIDE_02 actuator type with position limit 4π rad, velocity limit 44 rad/s, torque limit 17 Nm.

## Important Notes

1. **Binary path issue**: Two install paths exist. Always verify the running binary is from `~/VMC_Quadruped/install/` not `~/VMC_Quadruped/src/robstride_ros_sample/install/`

2. **Motor node behavior**: Node waits for first command on `/motor_command` before sending any CAN messages to motors

3. **Message includes**: Headers at `/home/lzh/VMC_Quadruped/install/interfaces/include/interfaces/interfaces/msg` for IDE intellisense
