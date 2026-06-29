import pathlib

import click

from plot_compare_dataset_scale_common import (
    build_series_lists,
    default_output_path,
    output_path_for_chunk,
    save_compare_2d,
    split_compare_chunks,
    title_for_chunk,
)


@click.command()
@click.option('--input', '-i', default='data', show_default=True, help='Session directory, or raw_videos/scale_calibration path.')
@click.option('--output', '-o', default=None, help='Output PNG path.')
@click.option('--include-lost', is_flag=True, default=False, help='Include lost frames in the plotted line.')
@click.option('--simultaneous', default=3, show_default=True, type=int, help='Number of dataset trajectories to plot together with all scale trajectories per image.')
def main(input, output, include_lost, simultaneous):
    session_dir, dataset_series, scale_series = build_series_lists(pathlib.Path(input))
    series_chunks = split_compare_chunks(dataset_series, scale_series, simultaneous)
    output_path = default_output_path(session_dir, 'plot_compare_dataset_scale_2d') if output is None else pathlib.Path(output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    for chunk_idx, chunk in enumerate(series_chunks):
        this_output_path = output_path_for_chunk(output_path, chunk_idx, len(series_chunks))
        save_compare_2d(
            series_list=chunk,
            output_path=this_output_path,
            title=title_for_chunk(f'Dataset + Scale Absolute XYZ Trajectories ({session_dir})', chunk_idx, len(series_chunks)),
            include_lost=include_lost,
        )
        print(f'Saved {this_output_path}')


if __name__ == '__main__':
    main()
