# scripts_real Overview

This directory contains real-robot entrypoints. The current target setup is
Franka single-arm policy/eval. The main eval entrypoint is planned at the
repository root as `eval_real.py`, and the real replay entrypoint is planned at
the repository root as `replay_real.py`.

## Terms

- `teleop`: live human control, usually with SpaceMouse. Used to move the robot by hand.
- `control`: utility command/test script for a robot or gripper. Usually no policy checkpoint.
- `demo`: human-controlled data collection. Records camera, robot, and gripper data for training.
- `eval`: runs a trained policy checkpoint on the real robot.
- `replay`: follows an already recorded dataset trajectory. No policy inference.

UMI demo/eval scripts usually assume the UMI camera/gripper setup used for data collection.
Franka deployment scripts are separate and need the Franka server plus the target gripper setup.

## Currently Used

| File | Purpose | Required setup |
| --- | --- | --- |
| `launch_franka_interface_server.py` | Franka zerorpc server | Franka host with Polymetis; no camera, gripper, or SpaceMouse |
| `* move_franka_joints.py` | Franka joint target CLI | Franka zerorpc server only |
| `* move_gripper_width.py` | DYROS UMI gripper width CLI | DYROS UMI gripper serial connection only |
| `* check_franka_tcp_frame.py` | Franka TCP/frame check script | Franka zerorpc server only |

`*`: additional file.

## Existing Files (Legacy)

| File | Purpose | Required setup |
| --- | --- | --- |
| `control_franka.py` | Franka + WSG + SpaceMouse teleop | Franka server, WSG gripper, SpaceMouse |
| `control_robot_spacemouse.py` | UR RTDE + WSG + SpaceMouse teleop | UR robot, WSG gripper, SpaceMouse |
| `control_robots.py` | UR/WSG robot control | UR robot, WSG gripper, SpaceMouse |
| `control_wsg_spacemouse.py` | WSG gripper SpaceMouse control | WSG gripper, SpaceMouse |
| `demo_real_bimanual_robots.py` | Bimanual real-robot demo | Two robots/grippers, cameras, SpaceMouse |
| `demo_real_robot.py` | Generic `RealEnv` demo | Generic UR/WSG-style `RealEnv`, cameras, SpaceMouse |
| `demo_real_umi.py` | Single-arm UMI SpaceMouse demo | UMI camera/gripper setup, robot, SpaceMouse |
| `eval_real_bimanual_umi.py` | Bimanual UMI policy eval | Policy checkpoint, bimanual UMI setup, cameras, SpaceMouse |
| `eval_real_robot.py` | Generic `RealEnv` policy eval | Policy checkpoint, generic `RealEnv`, cameras, SpaceMouse |
| `eval_real_umi.py` | Original single-arm UMI policy eval | Policy checkpoint, UMI setup, cameras, SpaceMouse |
| `eval_replay_real_robot.py` | Existing dataset trajectory replay through `UmiEnv` | Dataset/replay buffer, UMI-style env, robot/gripper, cameras |
| `latency_test_wsg.py` | WSG gripper latency test | WSG gripper, SpaceMouse |
| `replay_real_bimanual_umi.py` | Bimanual UMI dataset replay | Dataset/replay buffer, bimanual UMI setup, cameras, SpaceMouse |

## Notes

- The canonical policy eval path should be the root `eval_real.py`.
- The canonical real replay path should be the root `replay_real.py`.
- Files in this directory are kept as legacy/reference unless explicitly revived.
