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
from matplotlib.collections import LineCollection
from mpl_toolkits.mplot3d.art3d import Line3DCollection

from plot_dataset_common import load_demo_trajectory

SOURCE_COLORS = {
    'dataset': '#1f77b4',
    'scale': '#ff7f0e',
}

SOURCE_CMAPS = {
    'dataset': plt.cm.Blues,
    'scale': plt.cm.Oranges,
}


def resolve_session_dir(input_value: str) -> pathlib.Path:
    path = pathlib.Path(input_value).expanduser().resolve()
    if path.name == 'scale_calibration' and path.parent.name == 'raw_videos':
        path = path.parent.parent
    if path.is_file():
        path = path.parent
    dataset_demos_dir = path / 'demos'
    scale_demos_dir = dataset_demos_dir
    if not dataset_demos_dir.is_dir():
        raise click.ClickException(f'Missing dataset demos directory: {dataset_demos_dir}')
    return path


def _iter_camera_dirs(root: pathlib.Path, prefix: str):
    csv_paths = sorted(root.glob('**/camera_trajectory.csv'))
    dirs = []
    for csv_path in csv_paths:
        demo_dir = csv_path.parent
        if demo_dir.name.startswith(prefix):
            dirs.append(demo_dir)
    return dirs


def _segment_colors(source_name: str, n_segments: int):
    cmap = SOURCE_CMAPS[source_name]
    return cmap(np.linspace(0.95, 0.35, n_segments, endpoint=True))


def _load_source_series(demo_root: pathlib.Path, prefix: str, source_name: str):
    series_list = []
    for demo_dir in _iter_camera_dirs(demo_root, prefix):
        data = load_demo_trajectory(demo_dir)
        series_list.append({
            'source_name': source_name,
            'name': data['name'],
            'timestamps': data['timestamps'],
            'plot_positions': data['positions'],
            'is_lost': data['is_lost'],
        })
    return series_list


def build_series_lists(session_dir: pathlib.Path):
    session_dir = resolve_session_dir(str(session_dir))
    dataset_demos_dir = session_dir / 'demos'
    scale_demos_dir = dataset_demos_dir

    dataset_series = _load_source_series(dataset_demos_dir, 'demo_', 'dataset')
    scale_series = _load_source_series(scale_demos_dir, 'scale_calibration_', 'scale')

    if not dataset_series and not scale_series:
        raise click.ClickException(f'No dataset/scale camera_trajectory.csv found under {session_dir}')
    if not scale_series:
        raise click.ClickException(f'No scale calibration trajectories found under {scale_demos_dir}')
    return session_dir, dataset_series, scale_series


def split_compare_chunks(dataset_series, scale_series, simultaneous: int):
    if simultaneous <= 0:
        raise click.ClickException('--simultaneous must be a positive integer')
    if not dataset_series:
        return [list(scale_series)]
    chunks = []
    for i in range(0, len(dataset_series), simultaneous):
        chunks.append(list(scale_series) + dataset_series[i:i + simultaneous])
    return chunks


def output_path_for_chunk(output_path: pathlib.Path, chunk_idx: int, n_chunks: int):
    if n_chunks == 1:
        return output_path
    return output_path.with_name(f'{output_path.stem}_{chunk_idx + 1:02d}{output_path.suffix}')


def title_for_chunk(base_title: str, chunk_idx: int, n_chunks: int):
    if n_chunks == 1:
        return base_title
    return f'{base_title} [{chunk_idx + 1}/{n_chunks}]'


def default_output_path(session_dir: pathlib.Path, stem: str):
    output_dir = session_dir / 'plots'
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir / f'{stem}.png'


def save_compare_3d(series_list, output_path, title: str, include_lost: bool, show: bool):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    all_positions = []
    for series in series_list:
        positions = np.asarray(series['plot_positions'], dtype=np.float64).copy()
        if not include_lost:
            positions[series['is_lost']] = np.nan
        valid_positions = positions[~np.isnan(positions).any(axis=1)]
        if len(valid_positions) == 0:
            continue
        all_positions.append(valid_positions)
        source_name = series['source_name']
        if len(valid_positions) >= 2:
            segments = np.stack([valid_positions[:-1], valid_positions[1:]], axis=1)
            collection = Line3DCollection(
                segments,
                colors=_segment_colors(source_name, len(segments)),
                linewidths=1.6,
                alpha=0.9,
            )
            ax.add_collection3d(collection)
        ax.scatter(
            valid_positions[0, 0], valid_positions[0, 1], valid_positions[0, 2],
            s=28, color=SOURCE_COLORS[source_name], marker='o', edgecolors='black', linewidths=0.3,
        )
        ax.scatter(
            valid_positions[-1, 0], valid_positions[-1, 1], valid_positions[-1, 2],
            s=40, color=SOURCE_COLORS[source_name], marker='X', edgecolors='black', linewidths=0.3,
        )

    ax.set_title(title)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.legend(
        handles=[
            mlines.Line2D([], [], color=SOURCE_COLORS['dataset'], linewidth=2, label='dataset demos'),
            mlines.Line2D([], [], color=SOURCE_COLORS['scale'], linewidth=2, label='scale calibration'),
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


def save_compare_2d(series_list, output_path: pathlib.Path, title: str, include_lost: bool):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    axis_names = ('x', 'y', 'z')
    global_x_min = None
    global_x_max = None

    for axis_idx, axis_name in enumerate(axis_names):
        ax = axes[axis_idx]
        axis_y_min = None
        axis_y_max = None
        for series in series_list:
            positions = np.asarray(series['plot_positions'], dtype=np.float64).copy()
            if not include_lost:
                positions[series['is_lost']] = np.nan
            valid_mask = ~np.isnan(positions[:, axis_idx])
            xs = np.asarray(series['timestamps'], dtype=np.float64)
            ys = positions[:, axis_idx]
            source_name = series['source_name']
            if valid_mask.sum() >= 2:
                valid_xs = xs[valid_mask]
                valid_ys = ys[valid_mask]
                xy = np.column_stack([valid_xs, valid_ys])
                segments = np.stack([xy[:-1], xy[1:]], axis=1)
                collection = LineCollection(
                    segments,
                    colors=_segment_colors(source_name, len(segments)),
                    linewidths=1.2,
                    alpha=0.9,
                )
                ax.add_collection(collection)
                x_min = float(valid_xs.min())
                x_max = float(valid_xs.max())
                y_min = float(valid_ys.min())
                y_max = float(valid_ys.max())
            elif valid_mask.sum() == 1:
                valid_xs = xs[valid_mask]
                valid_ys = ys[valid_mask]
                ax.scatter(valid_xs, valid_ys, color=SOURCE_COLORS[source_name], s=12, alpha=0.9)
                x_min = x_max = float(valid_xs[0])
                y_min = y_max = float(valid_ys[0])
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
                pad = 1.0
            else:
                pad = 0.01 * (global_x_max - global_x_min)
            ax.set_xlim(global_x_min - pad, global_x_max + pad)
        if (axis_y_min is not None) and (axis_y_max is not None):
            if axis_y_min == axis_y_max:
                pad = 1e-3
            else:
                pad = 0.05 * (axis_y_max - axis_y_min)
            ax.set_ylim(axis_y_min - pad, axis_y_max + pad)

    axes[0].set_title(title)
    axes[-1].set_xlabel('time [s]')
    axes[0].legend(
        handles=[
            mlines.Line2D([], [], color=SOURCE_COLORS['dataset'], linewidth=2, label='dataset demos'),
            mlines.Line2D([], [], color=SOURCE_COLORS['scale'], linewidth=2, label='scale calibration'),
        ],
        loc='best',
    )
    fig.tight_layout()
    fig.savefig(output_path, dpi=200)
    plt.close(fig)
