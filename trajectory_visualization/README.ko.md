# Trajectory Visualization

UMI camera trajectory, scale calibration, camera/TCP frame consistency를 확인하기 위한 시각화 스크립트 모음. 데이터 생성이나 로봇 실행 entrypoint와 섞이지 않도록 `scripts/`가 아니라 별도 폴더로 분리함.

## Purpose

이 도구들의 주 용도는 camera trajectory, scale calibration, mapping, TCP trajectory의 개형과 scale이 예상대로 맞는지 확인하는 것. pipeline output이 생기는 순서에 맞춰 아래 순서로 확인하면 됨.

video processing과 mapping/demo/scale-calibration video의 SLAM이 끝난 뒤, 보통 pre-scale SLAM pipeline 또는 `00`-`03` 단계 이후에는 먼저 scale calibration과 mapping 결과를 확인:

```bash
python trajectory_visualization/plot_scale_3d.py --demo data/demos --simultaneous 3 --show

python trajectory_visualization/plot_scale_2d.py --demo data/demos --simultaneous 3 --show
```

같은 SLAM 단계 이후에는 robot-base anchor pose 기준으로 replay한 dataset demo camera motion 확인:

```bash
python trajectory_visualization/plot_dataset_real_demo_3d.py --demo data/demos --simultaneous 3 --show
```

`dataset_plan.pkl`을 만드는 post-processing 단계 이후에는 camera-derived TCP trajectory와 plan에 저장된 TCP trajectory 비교:

```bash
python trajectory_visualization/plot_compare_demo_pkl_3d.py -i data --simultaneous 3 --show
```

## Main Checks

| 실행 가능 시점 | Script | Purpose | Required inputs |
| --- | --- | --- | --- |
| video processing + SLAM 이후 (`00`-`03` / pre-scale pipeline) | `plot_scale_2d.py`, `plot_scale_3d.py` | scale-calibration과 mapping trajectory를 첫 frame 기준 relative frame으로 plot. scale calibration과 mapping 개형을 먼저 확인할 때 사용. | `demos/scale_calibration_*/camera_trajectory.csv`, `demos/mapping/camera_trajectory.csv` |
| video processing + SLAM 이후 (`00`-`03` / pre-scale pipeline) | `plot_compare_dataset_scale_2d.py`, `plot_compare_dataset_scale_3d.py` | dataset demo와 scale-calibration trajectory를 SLAM/map frame에서 비교. | `demo_*`, `scale_calibration_*` trajectory가 있는 `demos/` session directory |
| video processing + SLAM 이후 (`00`-`03` / pre-scale pipeline) | `plot_dataset_slam_2d.py`, `plot_dataset_slam_3d.py` | 원본 SLAM/map frame camera trajectory plot. | `demos/**/camera_trajectory.csv` |
| video processing + SLAM 이후 (`00`-`03` / pre-scale pipeline) | `plot_dataset_real_2d.py`, `plot_dataset_real_3d.py` | 각 camera trajectory를 자기 첫 frame 기준 relative trajectory로 plot. | `demos/**/camera_trajectory.csv` |
| video processing + SLAM 이후 (`00`-`03` / pre-scale pipeline) | `plot_dataset_real_demo_2d.py`, `plot_dataset_real_demo_3d.py` | relative camera motion을 robot-base anchor pose에서 replay한 형태로 plot. | `demos/**/camera_trajectory.csv` |
| dataset plan 생성 이후 (`06` / post-scale pipeline) | `plot_compare_demo_pkl_2d.py`, `plot_compare_demo_pkl_3d.py` | demo camera-derived trajectory와 `dataset_plan.pkl` TCP trajectory 비교. | `demos/`, `dataset_plan.pkl`가 있는 session directory |
| dataset plan 생성 이후 (`06` / post-scale pipeline) | `plot_debug_cam_tcp_components.py` | camera translation, rotated camera-to-TCP offset, reconstructed TCP, pkl TCP component debug. | `demos/`, `dataset_plan.pkl`, `demos/mapping/tx_slam_tag.json`가 있는 session directory |

## Frames

- `absolute SLAM/map frame`: `mapping.mp4`로 만든 ORB-SLAM global frame.
- `relative camera frame`: 첫 camera pose 기준으로 표현한 trajectory. 첫 sample이 항상 `(0, 0, 0)`.
- `robot-base anchored relative frame`: 선택한 robot TCP anchor pose에서 relative trajectory를 replay한 frame. `scripts_real/check_franka_tcp_frame.py`에서 출력되는 `ActualTCPPose`를 `DEFAULT_ANCHOR_POSE`로 쓰거나 `--anchor-pose`로 전달.
- `relative tag/world-aligned frame`: tag/world frame trajectory를 첫 sample이 local origin이 되도록 shift한 frame.

## Parameters

| Parameter | Scripts | Meaning | Default |
| --- | --- | --- | --- |
| `--demo` | `plot_dataset_slam_*`, `plot_dataset_real_*`, `plot_dataset_real_demo_*`, `plot_scale_*` | `camera_trajectory.csv`를 포함한 demo root. `plot_scale_*`는 `raw_videos/scale_calibration`도 받을 수 있음. | script별 기본값, 보통 `data/demo` 또는 `data/demos` |
| `--input`, `-i` | `plot_compare_dataset_scale_*`, `plot_compare_demo_pkl_*`, `plot_debug_cam_tcp_components.py` | session directory. pkl/debug plot은 `demos/`와 `dataset_plan.pkl` 필요. | debug는 `data/260429`, 나머지는 `data` |
| `--output`, `-o` | `plot_debug_cam_tcp_components.py` 제외한 2D/3D plot scripts | 출력 PNG 경로. 생략하면 입력 데이터 근처 `plots/` directory에 저장. | auto |
| `--include-lost` | dataset/scale trajectory plots | SLAM lost frame을 masking하지 않고 line에 포함. | off |
| `--simultaneous` | dataset/scale trajectory plots, pkl comparison plots | 이미지 하나에 같이 그릴 trajectory 또는 episode 수. | `3` |
| `--show` | 3D scripts, `plot_debug_cam_tcp_components.py` | matplotlib 창을 띄움. 일부 script는 이 옵션이 있으면 3D PNG 저장을 생략. | off |
| `--anchor-pose` | `plot_dataset_real_demo_*`, `plot_compare_demo_pkl_*` | Robot-base start TCP pose `[x y z rx ry rz]`. 기본값을 갱신할 때는 `scripts_real/check_franka_tcp_frame.py`의 `ActualTCPPose` 사용. | `DEFAULT_ANCHOR_POSE` |
| `--robot-idx` | `plot_compare_demo_pkl_*`, `plot_debug_cam_tcp_components.py` | `dataset_plan.pkl` 내부 robot/gripper index. | `0` |
| `--episode-idx` | `plot_debug_cam_tcp_components.py` | `dataset_plan.pkl` 내부 episode index. | `0` |
| `--tcp-offset` | `plot_debug_cam_tcp_components.py` | gripper tip에서 mounting screw까지의 거리. | `0.205` |
| `--cam-to-center-height` | `plot_debug_cam_tcp_components.py` | camera frame `+y` 방향에서 TCP center line까지의 offset. | `0.086` |
| `--cam-to-mount-offset` | `plot_debug_cam_tcp_components.py` | camera optical center에서 mount screw까지의 offset. | `0.01465` |
