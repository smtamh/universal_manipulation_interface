import json
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
from matplotlib.colors import to_rgba
from mpl_toolkits.mplot3d.art3d import Line3DCollection

from plot_dataset_common import load_demo_trajectory
from umi.common.pose_util import pose_to_mat


COMPONENT_COLORS = {
    'camera_translation': '#1f77b4',
    'rotated_offset': '#ff7f0e',
    'tcp_from_demo': '#2ca02c',
    'tcp_from_pkl': '#d62728',
}


def resolve_session_dir(input_value: str) -> pathlib.Path:
    path = pathlib.Path(input_value).expanduser().resolve()
    if path.is_file():
        if path.name != 'dataset_plan.pkl':
            raise click.ClickException(f'Expected session directory or dataset_plan.pkl, got {path}')
        path = path.parent
    demos_dir = path / 'demos'
    plan_path = path / 'dataset_plan.pkl'
    if not demos_dir.is_dir():
        raise click.ClickException(f'Missing demos directory: {demos_dir}')
    if not plan_path.is_file():
        raise click.ClickException(f'Missing dataset_plan.pkl: {plan_path}')
    return path


def _blend_with_white(color, mix: float):
    rgba = np.array(to_rgba(color), dtype=np.float64)
    white = np.array([1.0, 1.0, 1.0, rgba[3]], dtype=np.float64)
    return tuple(white * mix + rgba * (1.0 - mix))


def _segment_colors(base_color: str, n_segments: int):
    start_color = np.array(_blend_with_white(base_color, 0.75), dtype=np.float64)
    end_color = np.array(to_rgba(base_color), dtype=np.float64)
    if n_segments <= 1:
        return np.array([end_color])
    weights = np.linspace(1.0, 0.0, n_segments, endpoint=True)[:, None]
    return start_color * (1.0 - weights) + end_color * weights


def load_debug_data(session_dir: pathlib.Path, episode_idx: int, robot_idx: int, tcp_offset: float, cam_to_center_height: float, cam_to_mount_offset: float):
    session_dir = resolve_session_dir(str(session_dir))
    demos_dir = session_dir / 'demos'
    plan = pickle.load((session_dir / 'dataset_plan.pkl').open('rb'))
    if not (0 <= episode_idx < len(plan)):
        raise click.ClickException(f'episode_idx {episode_idx} out of range, total episodes={len(plan)}')
    episode = plan[episode_idx]
    if robot_idx >= len(episode['grippers']) or robot_idx >= len(episode['cameras']):
        raise click.ClickException(f'robot_idx {robot_idx} out of range for episode {episode_idx}')

    camera = episode['cameras'][robot_idx]
    video_rel = pathlib.Path(camera['video_path'])
    demo_dir = demos_dir / video_rel.parent
    frame_start, frame_end = camera['video_start_end']

    demo = load_demo_trajectory(demo_dir)
    demo_positions = np.asarray(demo['positions'][frame_start:frame_end], dtype=np.float64)
    demo_pose = np.asarray(demo['pose'][frame_start:frame_end], dtype=np.float64)
    timestamps = np.asarray(demo['timestamps'][frame_start:frame_end], dtype=np.float64)
    if len(demo_pose) == 0:
        raise click.ClickException(f'Empty demo segment for episode {episode_idx}: {demo_dir}[{frame_start}:{frame_end}]')

    tx_slam_tag_path = demos_dir / 'mapping' / 'tx_slam_tag.json'
    if not tx_slam_tag_path.is_file():
        raise click.ClickException(f'Missing tx_slam_tag.json: {tx_slam_tag_path}')
    tx_slam_tag = np.asarray(json.load(tx_slam_tag_path.open('r'))['tx_slam_tag'], dtype=np.float64)
    tx_tag_slam = np.linalg.inv(tx_slam_tag)

    tx_slam_cam = np.stack([pose_to_mat(pose) for pose in demo_pose], axis=0)
    tx_tag_cam = tx_tag_slam[None, ...] @ tx_slam_cam
    tag_cam_positions = tx_tag_cam[:, :3, 3]
    tag_cam_rot = tx_tag_cam[:, :3, :3]

    cam_to_tip_offset = cam_to_mount_offset + tcp_offset
    t_cam_tcp = np.array([0.0, cam_to_center_height, cam_to_tip_offset], dtype=np.float64)
    rotated_offset_abs = np.einsum('nij,j->ni', tag_cam_rot, t_cam_tcp)
    tcp_from_demo_abs = tag_cam_positions + rotated_offset_abs

    tcp_from_pkl_abs = np.asarray(episode['grippers'][robot_idx]['tcp_pose'], dtype=np.float64)[:, :3]

    return {
        'session_dir': session_dir,
        'demo_dir': demo_dir,
        'episode_idx': episode_idx,
        'robot_idx': robot_idx,
        'timestamps': timestamps,
        'camera_translation': tag_cam_positions - tag_cam_positions[0],
        'rotated_offset': rotated_offset_abs - rotated_offset_abs[0],
        'tcp_from_demo': tcp_from_demo_abs - tcp_from_demo_abs[0],
        'tcp_from_pkl': tcp_from_pkl_abs - tcp_from_pkl_abs[0],
        'raw_demo_positions_slam': demo_positions,
        't_cam_tcp': t_cam_tcp,
    }


def default_output_base(session_dir: pathlib.Path, episode_idx: int) -> pathlib.Path:
    output_dir = session_dir / 'plots'
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir / f'plot_debug_cam_tcp_components_ep{episode_idx:03d}'


def save_2d_plot(data: dict, output_path: pathlib.Path):
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    axis_names = ('x', 'y', 'z')
    global_x_min = None
    global_x_max = None

    for axis_idx, axis_name in enumerate(axis_names):
        ax = axes[axis_idx]
        axis_y_min = None
        axis_y_max = None
        for key, label in (
            ('camera_translation', 'camera position'),
            ('rotated_offset', 'R @ t contribution'),
            ('tcp_from_demo', 'camera + R @ t'),
            ('tcp_from_pkl', 'pkl tcp'),
        ):
            positions = np.asarray(data[key], dtype=np.float64)
            xs = np.arange(len(positions), dtype=np.float64)
            ys = positions[:, axis_idx]
            if len(positions) >= 2:
                xy = np.column_stack([xs, ys])
                segments = np.stack([xy[:-1], xy[1:]], axis=1)
                collection = LineCollection(
                    segments,
                    colors=_segment_colors(COMPONENT_COLORS[key], len(segments)),
                    linewidths=1.4,
                    alpha=0.95,
                )
                ax.add_collection(collection)
                x_min = float(xs.min())
                x_max = float(xs.max())
                y_min = float(ys.min())
                y_max = float(ys.max())
            elif len(positions) == 1:
                ax.scatter(xs, ys, color=COMPONENT_COLORS[key], s=12, alpha=0.95)
                x_min = x_max = float(xs[0])
                y_min = y_max = float(ys[0])
            else:
                continue

            global_x_min = x_min if global_x_min is None else min(global_x_min, x_min)
            global_x_max = x_max if global_x_max is None else max(global_x_max, x_max)
            axis_y_min = y_min if axis_y_min is None else min(axis_y_min, y_min)
            axis_y_max = y_max if axis_y_max is None else max(axis_y_max, y_max)

        ax.set_ylabel(f'{axis_name} [m]')
        ax.grid(True, alpha=0.3)
        if (global_x_min is not None) and (global_x_max is not None):
            if global_x_min == global_x_max:
                x_pad = 1.0
            else:
                x_pad = 0.01 * (global_x_max - global_x_min)
            ax.set_xlim(global_x_min - x_pad, global_x_max + x_pad)
        if (axis_y_min is not None) and (axis_y_max is not None):
            if axis_y_min == axis_y_max:
                y_pad = 1e-3
            else:
                y_pad = 0.05 * (axis_y_max - axis_y_min)
            ax.set_ylim(axis_y_min - y_pad, axis_y_max + y_pad)

    axes[0].set_title(
        f"Camera/TCP components relative to first frame\n"
        f"episode={data['episode_idx']} demo={data['demo_dir'].name} offset={data['t_cam_tcp']}"
    )
    axes[-1].set_xlabel('sample index')
    axes[0].legend(
        handles=[
            mlines.Line2D([], [], color=COMPONENT_COLORS['camera_translation'], linewidth=2, label='camera position'),
            mlines.Line2D([], [], color=COMPONENT_COLORS['rotated_offset'], linewidth=2, label='R @ t contribution'),
            mlines.Line2D([], [], color=COMPONENT_COLORS['tcp_from_demo'], linewidth=2, label='camera + R @ t'),
            mlines.Line2D([], [], color=COMPONENT_COLORS['tcp_from_pkl'], linewidth=2, label='pkl tcp'),
        ],
        loc='best',
    )
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)


def save_3d_plot(data: dict, output_path, show: bool):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    all_positions = []
    for key, label in (
        ('camera_translation', 'camera position'),
        ('rotated_offset', 'R @ t contribution'),
        ('tcp_from_demo', 'camera + R @ t'),
        ('tcp_from_pkl', 'pkl tcp'),
    ):
        positions = np.asarray(data[key], dtype=np.float64)
        all_positions.append(positions)
        if len(positions) >= 2:
            segments = np.stack([positions[:-1], positions[1:]], axis=1)
            collection = Line3DCollection(
                segments,
                colors=_segment_colors(COMPONENT_COLORS[key], len(segments)),
                linewidths=1.6,
                alpha=0.95,
            )
            ax.add_collection3d(collection)
        start_color = _blend_with_white(COMPONENT_COLORS[key], 0.75)
        ax.scatter(
            positions[0, 0], positions[0, 1], positions[0, 2],
            s=28, color=start_color, marker='o', edgecolors='black', linewidths=0.3,
        )
        ax.scatter(
            positions[-1, 0], positions[-1, 1], positions[-1, 2],
            s=40, color=COMPONENT_COLORS[key], marker='X', edgecolors='black', linewidths=0.3,
        )

    ax.set_title(
        f"Camera/TCP components relative to first frame\n"
        f"episode={data['episode_idx']} demo={data['demo_dir'].name}"
    )
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.legend(
        handles=[
            mlines.Line2D([], [], color=COMPONENT_COLORS['camera_translation'], linewidth=2, label='camera position'),
            mlines.Line2D([], [], color=COMPONENT_COLORS['rotated_offset'], linewidth=2, label='R @ t contribution'),
            mlines.Line2D([], [], color=COMPONENT_COLORS['tcp_from_demo'], linewidth=2, label='camera + R @ t'),
            mlines.Line2D([], [], color=COMPONENT_COLORS['tcp_from_pkl'], linewidth=2, label='pkl tcp'),
        ],
        loc='best',
    )

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


@click.command()
@click.option('--input', '-i', default='data/260429', show_default=True, help='Session directory containing demos/ and dataset_plan.pkl.')
@click.option('--episode-idx', default=0, show_default=True, type=int, help='Episode index in dataset_plan.pkl.')
@click.option('--robot-idx', default=0, show_default=True, type=int, help='Robot/gripper index.')
@click.option('--tcp-offset', default=0.205, show_default=True, type=float, help='Distance from gripper tip to mounting screw.')
@click.option('--cam-to-center-height', default=0.086, show_default=True, type=float, help='Camera-frame +y offset to TCP center line.')
@click.option('--cam-to-mount-offset', default=0.01465, show_default=True, type=float, help='Camera optical center to mount screw offset.')
@click.option('--show', is_flag=True, default=False, help='Open the 3D plot window without saving the 3D PNG.')
def main(input, episode_idx, robot_idx, tcp_offset, cam_to_center_height, cam_to_mount_offset, show):
    data = load_debug_data(
        session_dir=pathlib.Path(input),
        episode_idx=episode_idx,
        robot_idx=robot_idx,
        tcp_offset=tcp_offset,
        cam_to_center_height=cam_to_center_height,
        cam_to_mount_offset=cam_to_mount_offset,
    )
    output_base = default_output_base(data['session_dir'], episode_idx)
    save_2d_plot(data, output_base.with_name(output_base.name + '_2d.png'))
    save_3d_plot(data, None if show else output_base.with_name(output_base.name + '_3d.png'), show)
    print(f"Saved {output_base.with_name(output_base.name + '_2d.png')}")
    if not show:
        print(f"Saved {output_base.with_name(output_base.name + '_3d.png')}")


if __name__ == '__main__':
    main()
