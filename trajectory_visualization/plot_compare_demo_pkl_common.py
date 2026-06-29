import os
import pathlib
import pickle
import sys

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

import click
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import lines as mlines
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d.art3d import Line3DCollection

from plot_dataset_common import (
    apply_anchor_pose,
    convert_pose_to_relative_frame,
    load_demo_trajectory,
    output_path_for_chunk,
    split_series_list,
    title_for_chunk,
)

DEFAULT_ANCHOR_POSE = (
    0.74191,
    -0.06563,
    0.32997,
    -1.20938,
    1.20178,
    -1.22141,
)

SOURCE_COLORS = {
    'demo': '#1f77b4',
    'pkl': '#ff7f0e',
}

SOURCE_CMAPS = {
    'demo': plt.cm.Blues,
    'pkl': plt.cm.Oranges,
}


def resolve_session_dir(input_value: str) -> pathlib.Path:
    path = pathlib.Path(input_value).expanduser().resolve()
    if path.is_file():
        if path.name != 'dataset_plan.pkl':
            raise click.ClickException(f'Expected session dir or dataset_plan.pkl, got {path}')
        path = path.parent
    demos_dir = path / 'demos'
    plan_path = path / 'dataset_plan.pkl'
    if not demos_dir.is_dir():
        raise click.ClickException(f'Missing demos directory: {demos_dir}')
    if not plan_path.is_file():
        raise click.ClickException(f'Missing dataset_plan.pkl: {plan_path}')
    return path


def _build_demo_robot_segment(demo_dir: pathlib.Path, frame_start: int, frame_end: int, anchor_pose):
    data = load_demo_trajectory(demo_dir)
    pose = np.asarray(data['pose'][frame_start:frame_end], dtype=np.float64)
    timestamps = np.asarray(data['timestamps'][frame_start:frame_end], dtype=np.float64)
    if len(pose) == 0:
        raise click.ClickException(f'Empty demo segment for {demo_dir} [{frame_start}:{frame_end}]')
    rel_pose = convert_pose_to_relative_frame(pose)
    robot_pose = apply_anchor_pose(rel_pose, np.asarray(anchor_pose, dtype=np.float64))
    return {
        'positions': robot_pose[:, :3],
        'timestamps': timestamps,
    }


def _build_pkl_robot_segment(episode: dict, anchor_pose, robot_idx: int):
    grippers = episode['grippers']
    if robot_idx >= len(grippers):
        raise click.ClickException(f'robot_idx {robot_idx} out of range for episode')
    tcp_pose = np.asarray(grippers[robot_idx]['tcp_pose'], dtype=np.float64)
    timestamps = np.asarray(episode['episode_timestamps'], dtype=np.float64)
    rel_pose = convert_pose_to_relative_frame(tcp_pose)
    robot_pose = apply_anchor_pose(rel_pose, np.asarray(anchor_pose, dtype=np.float64))
    return {
        'positions': robot_pose[:, :3],
        'timestamps': timestamps,
    }


def build_compare_entries(session_dir: pathlib.Path, anchor_pose, robot_idx: int):
    session_dir = resolve_session_dir(str(session_dir))
    demos_dir = session_dir / 'demos'
    plan = pickle.load((session_dir / 'dataset_plan.pkl').open('rb'))

    entries = []
    for episode_idx, episode in enumerate(plan):
        cameras = episode['cameras']
        if robot_idx >= len(cameras):
            raise click.ClickException(f'robot_idx {robot_idx} out of range for episode {episode_idx}')
        camera = cameras[robot_idx]
        video_rel = pathlib.Path(camera['video_path'])
        demo_dir = demos_dir / video_rel.parent
        frame_start, frame_end = camera['video_start_end']

        demo_data = _build_demo_robot_segment(demo_dir, frame_start, frame_end, anchor_pose)
        pkl_data = _build_pkl_robot_segment(episode, anchor_pose, robot_idx)

        entries.append({
            'name': f'ep{episode_idx:03d}:{demo_dir.name}',
            'demo_positions': demo_data['positions'],
            'demo_timestamps': demo_data['timestamps'],
            'pkl_positions': pkl_data['positions'],
            'pkl_timestamps': pkl_data['timestamps'],
        })
    return entries


def default_output_path(session_dir: pathlib.Path, stem: str) -> pathlib.Path:
    output_dir = session_dir / 'plots'
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir / f'{stem}.png'


def _segment_colors(source_name: str, n_segments: int):
    cmap = SOURCE_CMAPS[source_name]
    return cmap(np.linspace(0.95, 0.35, n_segments, endpoint=True))

def save_compare_3d(entries, output_path, title: str, show: bool):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    all_positions = []
    for entry in entries:
        for source_name, color in SOURCE_COLORS.items():
            positions = np.asarray(entry[f'{source_name}_positions'], dtype=np.float64)
            if len(positions) == 0:
                continue
            all_positions.append(positions)
            if len(positions) >= 2:
                segments = np.stack([positions[:-1], positions[1:]], axis=1)
                collection = Line3DCollection(
                    segments,
                    colors=_segment_colors(source_name, len(segments)),
                    linewidths=1.6,
                    alpha=0.9,
                )
                ax.add_collection3d(collection)
            else:
                ax.scatter(
                    positions[:, 0],
                    positions[:, 1],
                    positions[:, 2],
                    color=color,
                    s=14,
                    alpha=0.9,
                )

    ax.set_title(title)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.legend(
        handles=[
            mlines.Line2D([], [], color=SOURCE_COLORS['demo'], linewidth=2, label='demo'),
            mlines.Line2D([], [], color=SOURCE_COLORS['pkl'], linewidth=2, label='pkl'),
        ],
        loc='best',
    )

    if all_positions:
        valid_points = np.concatenate(all_positions, axis=0)
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


def save_compare_2d(entries, output_path: pathlib.Path, title: str):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axis_names = ('x', 'y', 'z')

    for axis_idx, axis_name in enumerate(axis_names):
        ax = axes[axis_idx]
        for entry in entries:
            demo_positions = np.asarray(entry['demo_positions'], dtype=np.float64)
            pkl_positions = np.asarray(entry['pkl_positions'], dtype=np.float64)
            for source_name, positions in (
                ('demo', demo_positions),
                ('pkl', pkl_positions),
            ):
                if len(positions) == 0:
                    continue
                xs = np.arange(len(positions), dtype=np.float64)
                ys = positions[:, axis_idx]
                if len(positions) >= 2:
                    xy = np.column_stack([xs, ys])
                    segments = np.stack([xy[:-1], xy[1:]], axis=1)
                    collection = LineCollection(
                        segments,
                        colors=_segment_colors(source_name, len(segments)),
                        linewidths=1.2,
                        alpha=0.9,
                    )
                    ax.add_collection(collection)
                else:
                    ax.scatter(xs, ys, color=SOURCE_COLORS[source_name], s=12, alpha=0.9)
        ax.set_ylabel(f'{axis_name} [m]')
        ax.grid(True, alpha=0.3)
        ax.relim()
        ax.autoscale_view()

    axes[0].set_title(title)
    axes[-1].set_xlabel('sample index')
    axes[0].legend(
        handles=[
            mlines.Line2D([], [], color=SOURCE_COLORS['demo'], linewidth=2, label='demo'),
            mlines.Line2D([], [], color=SOURCE_COLORS['pkl'], linewidth=2, label='pkl'),
        ],
        loc='best',
    )
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)
