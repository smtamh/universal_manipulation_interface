import pathlib

import click

from plot_compare_demo_pkl_common import (
    DEFAULT_ANCHOR_POSE,
    build_compare_entries,
    default_output_path,
    resolve_session_dir,
    save_compare_3d,
)
from plot_dataset_common import output_path_for_chunk, split_series_list, title_for_chunk


@click.command()
@click.option('--input', '-i', default='data', show_default=True, help='Session directory containing demos/ and dataset_plan.pkl.')
@click.option('--output', '-o', default=None, help='Output PNG path.')
@click.option('--robot-idx', default=0, show_default=True, type=int, help='Robot/gripper index to compare.')
@click.option('--simultaneous', default=3, show_default=True, type=int, help='Number of episodes to plot per image.')
@click.option('--show', is_flag=True, default=False, help='Open the 3D matplotlib window without saving.')
@click.option('--anchor-pose', nargs=6, type=float, default=DEFAULT_ANCHOR_POSE, show_default=True, help='Robot-base replay start TCP pose [x y z rx ry rz].')
def main(input, output, robot_idx, simultaneous, show, anchor_pose):
    session_dir = resolve_session_dir(input)
    entries = build_compare_entries(session_dir, anchor_pose, robot_idx)
    entry_chunks = split_series_list(entries, simultaneous)
    output_path = default_output_path(session_dir, 'plot_compare_demo_pkl_3d') if output is None else pathlib.Path(output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    for chunk_idx, chunk in enumerate(entry_chunks):
        this_output_path = None if show else output_path_for_chunk(output_path, chunk_idx, len(entry_chunks))
        save_compare_3d(
            entries=chunk,
            output_path=this_output_path,
            title=title_for_chunk(f'Demo vs PKL Robot-Base 3D Trajectories ({session_dir.name})', chunk_idx, len(entry_chunks)),
            show=show,
        )
        if this_output_path is not None:
            print(f'Saved {this_output_path}')


if __name__ == '__main__':
    main()
