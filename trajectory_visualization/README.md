# Trajectory Visualization

Utilities for checking UMI camera trajectories, scale calibration, and camera/TCP frame consistency. These scripts were separated from `scripts/` so plotting/debug tools do not mix with data generation or robot entrypoints.

## Purpose

These tools are mainly for checking the shape and scale of camera trajectories and TCP trajectories after SLAM and dataset-plan generation. For the usual check, these two commands are enough:

```bash
python trajectory_visualization/plot_dataset_real_demo_3d.py --demo data/demos --simultaneous 3 --show

python trajectory_visualization/plot_compare_demo_pkl_3d.py -i data --simultaneous 3 --show
```

## Main Checks

| Script | Purpose | Required inputs |
| --- | --- | --- |
| `plot_dataset_slam_2d.py`, `plot_dataset_slam_3d.py` | Plot raw SLAM/map-frame camera trajectories. | `demos/**/camera_trajectory.csv` |
| `plot_compare_dataset_scale_2d.py`, `plot_compare_dataset_scale_3d.py` | Compare dataset demos and scale-calibration trajectories in the SLAM/map frame. | session directory with `demos/` |
| `plot_dataset_real_2d.py`, `plot_dataset_real_3d.py` | Plot each camera trajectory relative to its own first frame. | `demos/**/camera_trajectory.csv` |
| `plot_dataset_real_demo_2d.py`, `plot_dataset_real_demo_3d.py` | Replay relative camera motion from a robot-base anchor pose. | `demos/**/camera_trajectory.csv` |
| `plot_compare_demo_pkl_2d.py`, `plot_compare_demo_pkl_3d.py` | Compare demo camera-derived trajectory with `dataset_plan.pkl` TCP trajectory. | session directory with `demos/` and `dataset_plan.pkl` |
| `plot_debug_cam_tcp_components.py` | Debug camera translation, rotated camera-to-TCP offset, reconstructed TCP, and pkl TCP components. | session directory with `demos/`, `dataset_plan.pkl`, `demos/mapping/tx_slam_tag.json` |
| `plot_scale_2d.py`, `plot_scale_3d.py` | Plot scale-calibration and mapping trajectories relative to their first frame. | `demos/` or `raw_videos/scale_calibration` |

## Frames

- `absolute SLAM/map frame`: global frame from ORB-SLAM mapping.
- `relative camera frame`: trajectory expressed from the first camera pose, so the first sample is `(0, 0, 0)`.
- `robot-base anchored relative frame`: relative trajectory replayed from a chosen robot TCP anchor pose. Use the `ActualTCPPose` printed by `scripts_real/check_franka_tcp_frame.py` as `DEFAULT_ANCHOR_POSE` or pass it with `--anchor-pose`.
- `relative tag/world-aligned frame`: tag/world-frame motion shifted so the first sample is the local origin.

## Parameters

| Parameter | Scripts | Meaning | Default |
| --- | --- | --- | --- |
| `--demo` | `plot_dataset_slam_*`, `plot_dataset_real_*`, `plot_dataset_real_demo_*`, `plot_scale_*` | Demo root containing `camera_trajectory.csv` files. `plot_scale_*` also accepts `raw_videos/scale_calibration`. | script-specific, usually `data/demo` or `data/demos` |
| `--input`, `-i` | `plot_compare_dataset_scale_*`, `plot_compare_demo_pkl_*`, `plot_debug_cam_tcp_components.py` | Session directory. For pkl/debug plots, it must contain `demos/` and `dataset_plan.pkl`. | `data` except debug default `data/260429` |
| `--output`, `-o` | all 2D/3D plot scripts except `plot_debug_cam_tcp_components.py` | Output PNG path. If omitted, writes into a `plots/` directory near the input data. | auto |
| `--include-lost` | dataset/scale trajectory plots | Include SLAM lost frames instead of masking them from the line. | off |
| `--simultaneous` | dataset/scale trajectory plots and pkl comparison plots | Number of trajectories or episodes to draw per image. | `3` |
| `--show` | 3D scripts and `plot_debug_cam_tcp_components.py` | Open the matplotlib window. Some scripts skip saving the 3D PNG when this is set. | off |
| `--anchor-pose` | `plot_dataset_real_demo_*`, `plot_compare_demo_pkl_*` | Robot-base start TCP pose `[x y z rx ry rz]`. Use `ActualTCPPose` from `scripts_real/check_franka_tcp_frame.py` when updating the default. | `DEFAULT_ANCHOR_POSE` |
| `--robot-idx` | `plot_compare_demo_pkl_*`, `plot_debug_cam_tcp_components.py` | Robot/gripper index inside `dataset_plan.pkl`. | `0` |
| `--episode-idx` | `plot_debug_cam_tcp_components.py` | Episode index inside `dataset_plan.pkl`. | `0` |
| `--tcp-offset` | `plot_debug_cam_tcp_components.py` | Distance from gripper tip to mounting screw. | `0.205` |
| `--cam-to-center-height` | `plot_debug_cam_tcp_components.py` | Camera-frame `+y` offset to TCP center line. | `0.086` |
| `--cam-to-mount-offset` | `plot_debug_cam_tcp_components.py` | Camera optical center to mount screw offset. | `0.01465` |
