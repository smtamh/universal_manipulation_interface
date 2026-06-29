import pathlib

import click

from plot_dataset_common import (
    build_tag_series,
    ensure_output_path,
    find_demo_dirs,
    output_path_for_chunk,
    save_2d_plot,
    split_series_list,
    title_for_chunk,
)


@click.command()
@click.option("--demo", default="data/demo", show_default=True, help="Demo root directory.")
@click.option("--output", "-o", default=None, help="Output PNG path.")
@click.option("--include-lost", is_flag=True, default=False, help="Include lost frames in the plotted line.")
@click.option("--simultaneous", default=3, show_default=True, type=int, help="Number of demo trajectories to plot per image.")
def main(demo, output, include_lost, simultaneous):
    demo_root = pathlib.Path(demo)
    series_list = [build_tag_series(demo_dir) for demo_dir in find_demo_dirs(demo_root)]
    series_chunks = split_series_list(series_list, simultaneous)
    output_path = ensure_output_path(output, demo_root, "plot_dataset_tag_2d")

    for chunk_idx, chunk in enumerate(series_chunks):
        this_output_path = output_path_for_chunk(output_path, chunk_idx, len(series_chunks))
        save_2d_plot(
            series_list=chunk,
            output_path=this_output_path,
            include_lost=include_lost,
            title=title_for_chunk(f"Tag-Frame XYZ Trajectories ({demo_root})", chunk_idx, len(series_chunks)),
        )
        print(f"Saved {this_output_path}")


if __name__ == "__main__":
    main()
