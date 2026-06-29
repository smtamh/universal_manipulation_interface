import pathlib

import click

from plot_dataset_common import (
    build_real_series,
    ensure_output_path,
    find_demo_dirs,
    output_path_for_chunk,
    save_2d_plot,
    split_series_list,
    title_for_chunk,
)


def resolve_scale_demo_root(demo_value: str) -> pathlib.Path:
    path = pathlib.Path(demo_value).expanduser().resolve()
    if path.name == 'scale_calibration' and path.parent.name == 'raw_videos':
        return path.parent.parent / 'demos'
    return path


@click.command()
@click.option('--demo', default='data/demos', show_default=True, help='Demos root directory, or raw_videos/scale_calibration path.')
@click.option('--output', '-o', default=None, help='Output PNG path.')
@click.option('--include-lost', is_flag=True, default=False, help='Include lost frames in the plotted line.')
@click.option('--simultaneous', default=3, show_default=True, type=int, help='Number of scale trajectories to plot per image.')
def main(demo, output, include_lost, simultaneous):
    demo_root = resolve_scale_demo_root(demo)
    series_list = [
        build_real_series(demo_dir)
        for demo_dir in find_demo_dirs(demo_root, include_mapping=True)
        if demo_dir.name.startswith('scale_calibration_') or demo_dir.name == 'mapping'
    ]
    series_chunks = split_series_list(series_list, simultaneous)
    output_path = ensure_output_path(output, demo_root, 'plot_scale_2d')

    for chunk_idx, chunk in enumerate(series_chunks):
        this_output_path = output_path_for_chunk(output_path, chunk_idx, len(series_chunks))
        save_2d_plot(
            series_list=chunk,
            output_path=this_output_path,
            include_lost=include_lost,
            title=title_for_chunk(f'Relative Scale XYZ Trajectories ({demo_root})', chunk_idx, len(series_chunks)),
        )
        print(f'Saved {this_output_path}')


if __name__ == '__main__':
    main()
