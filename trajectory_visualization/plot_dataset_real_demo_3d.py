import pathlib

import click

from plot_dataset_common import (
    build_robot_series,
    ensure_output_path,
    find_demo_dirs,
    output_path_for_chunk,
    save_3d_plot,
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


@click.command()
@click.option("--demo", default="data/demo", show_default=True, help="Demo root directory.")
@click.option("--output", "-o", default=None, help="Output PNG path.")
@click.option("--include-lost", is_flag=True, default=False, help="Include lost frames in the plotted line.")
@click.option("--simultaneous", default=3, show_default=True, type=int, help="Number of demo trajectories to plot per image.")
@click.option("--show", is_flag=True, default=False, help="Open the 3D matplotlib window without saving.")
@click.option("--anchor-pose", nargs=6, type=float, default=DEFAULT_ANCHOR_POSE, show_default=True, help="Robot-base replay start TCP pose [x y z rx ry rz].")
def main(demo, output, include_lost, simultaneous, show, anchor_pose):
    demo_root = pathlib.Path(demo)
    series_list = [build_robot_series(demo_dir, anchor_pose) for demo_dir in find_demo_dirs(demo_root)]
    series_chunks = split_series_list(series_list, simultaneous)
    output_path = ensure_output_path(output, demo_root, "plot_dataset_real_demo_3d")

    for chunk_idx, chunk in enumerate(series_chunks):
        this_output_path = None if show else output_path_for_chunk(output_path, chunk_idx, len(series_chunks))
        save_3d_plot(
            series_list=chunk,
            output_path=this_output_path,
            include_lost=include_lost,
            title=title_for_chunk(f"Real Demo Robot-Base 3D Trajectories ({demo_root})", chunk_idx, len(series_chunks)),
            show=show,
        )
        if this_output_path is not None:
            print(f"Saved {this_output_path}")


if __name__ == "__main__":
    main()
