# scripts_real 개요

실제 로봇 실행용 entrypoint 모음. 현재 목표 세팅은 Franka single-arm
policy/eval이고, 주 eval entrypoint는 repository root의 `eval_real.py`로 정리 예정.
실제 로봇 replay entrypoint는 repository root의 `replay_real.py`로 정리 예정.

## 용어

- `teleop`: 사람이 SpaceMouse 같은 입력 장치로 로봇을 실시간 조종.
- `control`: 로봇/그리퍼에 특정 명령을 보내는 테스트/유틸 스크립트. 보통 policy checkpoint 없음.
- `demo`: 사람이 조종하면서 학습용 데이터를 수집. camera, robot, gripper 기록.
- `eval`: 학습된 policy checkpoint를 실제 로봇에서 실행.
- `replay`: 이미 저장된 dataset trajectory를 다시 따라감. policy inference 없음.

UMI demo/eval 스크립트는 보통 데이터 수집용 UMI camera/gripper 세팅을 전제로 함.
Franka 실제 배포용 스크립트는 별도이며 Franka server와 목표 gripper 세팅이 필요.

## 현재 사용 파일

| 파일 | 용도 | 필요한 세팅 |
| --- | --- | --- |
| `launch_franka_interface_server.py` | Franka zerorpc server | Polymetis가 붙은 Franka host. camera, gripper, SpaceMouse 불필요 |
| `* move_franka_joints.py` | Franka joint target CLI | Franka zerorpc server만 필요 |
| `* move_gripper_width.py` | DYROS UMI gripper width CLI | DYROS UMI gripper serial 연결만 필요 |
| `* check_franka_tcp_frame.py` | Franka TCP/frame 확인 | Franka zerorpc server만 필요 |

`*`: 추가된 파일.

## 기존 파일 목록 (legacy)

| 파일 | 용도 | 필요한 세팅 |
| --- | --- | --- |
| `control_franka.py` | Franka + WSG + SpaceMouse teleop | Franka server, WSG gripper, SpaceMouse |
| `control_robot_spacemouse.py` | UR RTDE + WSG + SpaceMouse teleop | UR robot, WSG gripper, SpaceMouse |
| `control_robots.py` | UR/WSG 로봇 제어 | UR robot, WSG gripper, SpaceMouse |
| `control_wsg_spacemouse.py` | WSG gripper SpaceMouse 제어 | WSG gripper, SpaceMouse |
| `demo_real_bimanual_robots.py` | bimanual 실제 로봇 demo | robot/gripper 2세트, cameras, SpaceMouse |
| `demo_real_robot.py` | generic `RealEnv` demo | generic UR/WSG 계열 `RealEnv`, cameras, SpaceMouse |
| `demo_real_umi.py` | single-arm UMI SpaceMouse demo | UMI camera/gripper 세팅, robot, SpaceMouse |
| `eval_real_bimanual_umi.py` | bimanual UMI policy eval | policy checkpoint, bimanual UMI 세팅, cameras, SpaceMouse |
| `eval_real_robot.py` | generic `RealEnv` policy eval | policy checkpoint, generic `RealEnv`, cameras, SpaceMouse |
| `eval_real_umi.py` | 기존 single-arm UMI policy eval | policy checkpoint, UMI 세팅, cameras, SpaceMouse |
| `eval_replay_real_robot.py` | 기존 `UmiEnv` 기반 dataset trajectory replay | dataset/replay buffer, UMI 계열 env, robot/gripper, cameras |
| `latency_test_wsg.py` | WSG gripper latency test | WSG gripper, SpaceMouse |
| `replay_real_bimanual_umi.py` | bimanual UMI dataset replay | dataset/replay buffer, bimanual UMI 세팅, cameras, SpaceMouse |

## 메모

- canonical policy eval 경로는 root `eval_real.py`.
- canonical real replay 경로는 root `replay_real.py`.
- 이 디렉토리의 파일들은 명시적으로 다시 살리기 전까지 legacy/reference.
