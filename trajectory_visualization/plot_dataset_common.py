import csv
import os
import pathlib
import sys

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

import click
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import lines as mlines
from mpl_toolkits.mplot3d.art3d import Line3DCollection
from scipy.spatial.transform import Rotation

from umi.common.pose_util import mat_to_pose, pose_to_mat


def find_demo_dirs(demo_root: pathlib.Path, include_mapping: bool = False):
    demo_root = demo_root.expanduser().resolve()
    csv_paths = sorted(demo_root.glob("**/camera_trajectory.csv"))
    demo_dirs = []
    for csv_path in csv_paths:
        demo_dir = csv_path.parent
        if (not include_mapping) and demo_dir.name == 'mapping':
            continue
        demo_dirs.append(demo_dir)
    if not demo_dirs:
        raise click.ClickException(f"No eligible camera_trajectory.csv found under {demo_root}")
    return demo_dirs


def load_demo_trajectory(demo_dir: pathlib.Path):
    demo_dir = demo_dir.expanduser().resolve()
    csv_path = demo_dir / "camera_trajectory.csv"
    if not csv_path.is_file():
        raise click.ClickException(f"Missing camera_trajectory.csv: {csv_path}")

    rows = []
    with csv_path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        required_cols = ["timestamp", "is_lost", "x", "y", "z", "q_x", "q_y", "q_z", "q_w"]
        if reader.fieldnames is None:
            raise click.ClickException(f"{csv_path} is empty")
        missing_cols = [col for col in required_cols if col not in reader.fieldnames]
        if missing_cols:
            raise click.ClickException(f"{csv_path} is missing required columns: {missing_cols}")
        for row in reader:
            rows.append(row)

    positions = np.array(
        [[float(row["x"]), float(row["y"]), float(row["z"])] for row in rows],
        dtype=np.float64,
    )
    quats = np.array(
        [
            [float(row["q_x"]), float(row["q_y"]), float(row["q_z"]), float(row["q_w"])]
            for row in rows
        ],
        dtype=np.float64,
    )
    timestamps = np.array([float(row["timestamp"]) for row in rows], dtype=np.float64)
    is_lost = np.array([row["is_lost"].strip().lower() == "true" for row in rows], dtype=bool)

    # Some SLAM CSVs contain zero-norm quaternions on lost frames. Treat them as identity
    # so plotting can continue while the lost-frame mask still hides them by default.
    quat_norms = np.linalg.norm(quats, axis=1)
    invalid_quat = quat_norms <= 1e-12
    quats[invalid_quat] = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    quats[~invalid_quat] = quats[~invalid_quat] / quat_norms[~invalid_quat, None]

    rotvecs = Rotation.from_quat(quats).as_rotvec()
    pose = np.concatenate([positions, rotvecs], axis=-1)

    return {
        "name": demo_dir.name,
        "demo_dir": demo_dir,
        "timestamps": timestamps,
        "positions": positions,
        "pose": pose,
        "is_lost": is_lost,
    }


def convert_pose_to_relative_frame(pose: np.ndarray):
    start_pose_mat = pose_to_mat(pose[0])
    rel_pose = np.zeros_like(pose)
    for idx, this_pose in enumerate(pose):
        rel_pose_mat = np.linalg.inv(start_pose_mat) @ pose_to_mat(this_pose)
        rel_pose[idx] = mat_to_pose(rel_pose_mat)
    return rel_pose


def apply_anchor_pose(relative_pose: np.ndarray, anchor_pose: np.ndarray):
    anchor_pose = np.asarray(anchor_pose, dtype=np.float64)
    anchor_pose_mat = pose_to_mat(anchor_pose)
    out_pose = np.zeros_like(relative_pose)
    for idx, this_pose in enumerate(relative_pose):
        out_pose[idx] = mat_to_pose(anchor_pose_mat @ pose_to_mat(this_pose))
    return out_pose


def build_tag_series(demo_dir: pathlib.Path):
    data = load_demo_trajectory(demo_dir)
    return {
        **data,
        "frame_name": "tag",
        "plot_positions": data["positions"],
    }


def build_real_series(demo_dir: pathlib.Path):
    data = load_demo_trajectory(demo_dir)
    rel_pose = convert_pose_to_relative_frame(data["pose"])
    return {
        **data,
        "frame_name": "real",
        "plot_positions": rel_pose[:, :3],
    }


def build_robot_series(demo_dir: pathlib.Path, anchor_pose):
    data = load_demo_trajectory(demo_dir)
    rel_pose = convert_pose_to_relative_frame(data["pose"])
    robot_pose = apply_anchor_pose(rel_pose, np.asarray(anchor_pose, dtype=np.float64))
    return {
        **data,
        "frame_name": "robot",
        "plot_positions": robot_pose[:, :3],
    }


def prepare_line_positions(series, include_lost: bool):
    positions = np.asarray(series["plot_positions"], dtype=np.float64).copy()
    if not include_lost:
        positions[series["is_lost"]] = np.nan
    return positions


def ensure_output_path(output, demo_root: pathlib.Path, script_stem: str):
    if output is None:
        output_dir = demo_root.expanduser().resolve().parent / "plots"
        output_dir.mkdir(parents=True, exist_ok=True)
        return output_dir / f"{script_stem}.png"
    output_path = pathlib.Path(output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    return output_path


def split_series_list(series_list, simultaneous: int):
    if simultaneous <= 0:
        raise click.ClickException("--simultaneous must be a positive integer")
    return [series_list[i:i + simultaneous] for i in range(0, len(series_list), simultaneous)]


def output_path_for_chunk(output_path: pathlib.Path, chunk_idx: int, n_chunks: int):
    if n_chunks == 1:
        return output_path
    return output_path.with_name(
        f"{output_path.stem}_{chunk_idx + 1:02d}{output_path.suffix}"
    )


def title_for_chunk(base_title: str, chunk_idx: int, n_chunks: int):
    if n_chunks == 1:
        return base_title
    return f"{base_title} [{chunk_idx + 1}/{n_chunks}]"


def save_3d_plot(series_list, output_path, include_lost: bool, title: str, show: bool = False):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    valid_points_list = []
    for series in series_list:
        positions = prepare_line_positions(series, include_lost=include_lost)
        valid_positions = positions[~np.isnan(positions).any(axis=1)]
        if len(valid_positions) == 0:
            continue
        valid_points_list.append(valid_positions)

        if len(valid_positions) >= 2:
            segments = np.stack([valid_positions[:-1], valid_positions[1:]], axis=1)
            segment_colors = plt.cm.viridis(
                np.linspace(0.0, 1.0, len(segments), endpoint=True)
            )
            collection = Line3DCollection(
                segments,
                colors=segment_colors,
                linewidths=1.8,
            )
            ax.add_collection3d(collection)

        ax.scatter(
            valid_positions[0, 0],
            valid_positions[0, 1],
            valid_positions[0, 2],
            s=45,
            marker="o",
            color="#2ca02c",
            edgecolors="black",
            linewidths=0.4,
            zorder=3,
        )
        ax.scatter(
            valid_positions[-1, 0],
            valid_positions[-1, 1],
            valid_positions[-1, 2],
            s=55,
            marker="X",
            color="#d62728",
            edgecolors="black",
            linewidths=0.4,
            zorder=3,
        )

    ax.set_title(title)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")

    gradient_handle = mlines.Line2D([], [], color=plt.cm.viridis(0.65), linewidth=2, label="trajectory")
    start_handle = mlines.Line2D(
        [], [], color="#2ca02c", marker="o", linestyle="None",
        markersize=7, markeredgecolor="black", markeredgewidth=0.4, label="start"
    )
    end_handle = mlines.Line2D(
        [], [], color="#d62728", marker="X", linestyle="None",
        markersize=8, markeredgecolor="black", markeredgewidth=0.4, label="end"
    )
    ax.legend(handles=[gradient_handle, start_handle, end_handle], loc="best")

    if valid_points_list:
        valid_points = np.concatenate(valid_points_list, axis=0)
        mins = valid_points.min(axis=0)
        maxs = valid_points.max(axis=0)
        centers = (mins + maxs) / 2.0
        radius = np.max(maxs - mins) / 2.0
        if radius > 0:
            ax.set_xlim(centers[0] - radius, centers[0] + radius)
            ax.set_ylim(centers[1] - radius, centers[1] + radius)
            ax.set_zlim(centers[2] - radius, centers[2] + radius)

    fig.tight_layout()
    if output_path is not None:
        fig.savefig(output_path, dpi=200)
    if show:
        plt.show()
    plt.close(fig)


def save_2d_plot(series_list, output_path: pathlib.Path, include_lost: bool, title: str):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axis_names = ("x", "y", "z")

    for axis_idx, axis_name in enumerate(axis_names):
        ax = axes[axis_idx]
        for series in series_list:
            positions = prepare_line_positions(series, include_lost=include_lost)
            ax.plot(
                series["timestamps"],
                positions[:, axis_idx],
                linewidth=1.2,
                label=series["name"],
            )
        ax.set_ylabel(f"{axis_name} [m]")
        ax.grid(True, alpha=0.3)

    axes[0].set_title(title)
    axes[-1].set_xlabel("time [s]")
    axes[0].legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)
