"""
Attach scale-calibration videos to the main demos workspace and localize them against the
shared dataset map.

This pipeline now:
- prepares / updates the main dataset workspace under <session_dir>/demos
- creates or reuses a shared map at <session_dir>/demos/mapping/map_atlas.osa
- runs batch SLAM for dataset demos against that shared map
- moves raw_videos/scale_calibration/*.mp4 into demos/scale_calibration_* directories
  with symlinks back to raw_videos, matching the gripper-calibration behavior
- runs IMU extraction and batch SLAM for those scale_calibration_* demos against the same map

The scale-calibration demos live in data/demos but are excluded from the normal dataset
pipeline by default.
"""

import sys
import os

ROOT_DIR = os.path.dirname(__file__)
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

import pathlib
import shutil
import click
import subprocess
from exiftool import ExifToolHelper

from umi.common.timecode_util import mp4_get_start_datetime

SCALE_CAL_PREFIX = 'scale_calibration_'


def list_mp4s(root: pathlib.Path):
    return sorted(list(root.glob('**/*.MP4')) + list(root.glob('**/*.mp4')))


def make_video_dir_name(prefix: str, mp4_path: pathlib.Path, exif: ExifToolHelper):
    start_date = mp4_get_start_datetime(str(mp4_path))
    meta = list(exif.get_metadata(str(mp4_path)))[0]
    cam_serial = meta['QuickTime:CameraSerialNumber']
    return prefix + cam_serial + '_' + start_date.strftime(r"%Y.%m.%d_%H.%M.%S.%f")


def run_cmd(cmd):
    result = subprocess.run(cmd)
    assert result.returncode == 0


def ensure_symlink_back(src_path: pathlib.Path, dst_path: pathlib.Path):
    dst_path.parent.mkdir(parents=True, exist_ok=True)

    if src_path.is_symlink():
        try:
            if src_path.resolve() == dst_path.resolve() and dst_path.exists():
                return dst_path
        except FileNotFoundError:
            src_path.unlink()
        else:
            src_path.unlink()

    if src_path.exists() and dst_path.exists():
        src_path.unlink()
        rel_target = os.path.relpath(dst_path, src_path.parent)
        src_path.symlink_to(rel_target)
        return dst_path

    if src_path.exists():
        shutil.move(str(src_path), str(dst_path))
    elif not dst_path.exists():
        raise RuntimeError(f'Missing both source and destination for scale video: {src_path}, {dst_path}')

    rel_target = os.path.relpath(dst_path, src_path.parent)
    src_path.symlink_to(rel_target)
    return dst_path


@click.command()
@click.argument('session_dir', nargs=-1)
@click.option('--no_docker_pull', is_flag=True, default=False, help='Forwarded to docker-based stages.')
def main(session_dir, no_docker_pull):
    repo_dir = pathlib.Path(__file__).parent
    script_dir = repo_dir.joinpath('scripts_slam_pipeline')

    for session in session_dir:
        session = pathlib.Path(os.path.expanduser(session)).absolute()
        raw_videos_dir = session.joinpath('raw_videos')
        if not raw_videos_dir.is_dir():
            raise RuntimeError(f'Missing raw_videos directory: {raw_videos_dir}')

        mapping_candidates = [
            raw_videos_dir.joinpath('mapping.mp4'),
            raw_videos_dir.joinpath('mapping.MP4'),
        ]
        mapping_path = next((p for p in mapping_candidates if p.is_file()), None)
        if mapping_path is None:
            raise RuntimeError(
                f"Missing mapping video. Expected one of: {', '.join(str(p) for p in mapping_candidates)}"
            )

        scale_source_dir = raw_videos_dir.joinpath('scale_calibration')
        if not scale_source_dir.is_dir():
            raise RuntimeError(f'Missing scale_calibration directory: {scale_source_dir}')
        scale_mp4_paths = list_mp4s(scale_source_dir)
        if len(scale_mp4_paths) == 0:
            raise RuntimeError(f'No mp4 found under {scale_source_dir}')

        print('############# Prepare shared dataset workspace ###########')
        cmd = ['python', str(script_dir.joinpath('00_process_videos.py')), str(session)]
        run_cmd(cmd)

        print('############# 01_extract_gopro_imu (dataset workspace) ###########')
        cmd = ['python', str(script_dir.joinpath('01_extract_gopro_imu.py'))]
        if no_docker_pull:
            cmd.append('--no_docker_pull')
        cmd.append(str(session))
        run_cmd(cmd)

        print('############# 02_create_map (shared dataset map) ###########')
        dataset_demos_dir = session.joinpath('demos')
        mapping_dir = dataset_demos_dir.joinpath('mapping')
        shared_map_path = mapping_dir.joinpath('map_atlas.osa')
        if shared_map_path.is_file():
            print(f'Reusing existing shared map: {shared_map_path}')
        else:
            cmd = [
                'python', str(script_dir.joinpath('02_create_map.py')),
                '--input_dir', str(mapping_dir),
                '--map_path', str(shared_map_path),
            ]
            if no_docker_pull:
                cmd.append('--no_docker_pull')
            run_cmd(cmd)

        print('############# 03_batch_slam (dataset demos on shared map) ###########')
        cmd = [
            'python', str(script_dir.joinpath('03_batch_slam.py')),
            '--input_dir', str(dataset_demos_dir),
            '--map_path', str(shared_map_path),
        ]
        if no_docker_pull:
            cmd.append('--no_docker_pull')
        run_cmd(cmd)

        with ExifToolHelper() as et:
            for mp4_path in scale_mp4_paths:
                out_dname = make_video_dir_name(SCALE_CAL_PREFIX, mp4_path, et)
                dst_path = dataset_demos_dir.joinpath(out_dname, 'raw_video.mp4')
                ensure_symlink_back(mp4_path, dst_path)

        print('############# 01_extract_gopro_imu (scale_calibration demos) ###########')
        cmd = ['python', str(script_dir.joinpath('01_extract_gopro_imu.py'))]
        if no_docker_pull:
            cmd.append('--no_docker_pull')
        cmd.append('--include_scale_calibration')
        cmd.append(str(session))
        run_cmd(cmd)

        print('############# 03_batch_slam (scale_calibration demos on shared map) ###########')
        cmd = [
            'python', str(script_dir.joinpath('03_batch_slam.py')),
            '--input_dir', str(dataset_demos_dir),
            '--map_path', str(shared_map_path),
            '--include_scale_calibration',
        ]
        if no_docker_pull:
            cmd.append('--no_docker_pull')
        run_cmd(cmd)

        print('Shared-map scale calibration demos ready:')
        print(f'  dataset demos: {dataset_demos_dir}')
        print(f'  scale demos:   {dataset_demos_dir}/scale_calibration_*')
        print(f'  shared map:    {shared_map_path}')


if __name__ == '__main__':
    main()
