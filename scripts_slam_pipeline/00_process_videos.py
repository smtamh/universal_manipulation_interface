"""
python scripts_slam_pipeline/00_process_videos.py data_workspace/toss_objects/20231113
"""
# %%
import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

# %%
import pathlib
import click
import shutil
from exiftool import ExifToolHelper
from umi.common.timecode_util import mp4_get_start_datetime


IGNORED_RAW_VIDEO_DIR_PREFIXES = (
    'scale_calibration',
)


def _is_ignored_raw_video_path(input_dir: pathlib.Path, mp4_path: pathlib.Path) -> bool:
    rel_parts = mp4_path.relative_to(input_dir).parts[:-1]
    for part in rel_parts:
        if any(part.startswith(prefix) for prefix in IGNORED_RAW_VIDEO_DIR_PREFIXES):
            return True
    return False


def _iter_input_mp4_paths(input_dir: pathlib.Path):
    mp4_paths = list(input_dir.glob('**/*.MP4')) + list(input_dir.glob('**/*.mp4'))
    return [path for path in mp4_paths if not _is_ignored_raw_video_path(input_dir, path)]


# %%
@click.command(help='Session directories. Assumming mp4 videos are in <session_dir>/raw_videos')
@click.argument('session_dir', nargs=-1)
def main(session_dir):
    for session in session_dir:
        session = pathlib.Path(os.path.expanduser(session)).absolute()
        # hardcode subdirs
        input_dir = session.joinpath('raw_videos')
        output_dir = session.joinpath('demos')
        
        # create raw_videos if don't exist
        if not input_dir.is_dir():
            input_dir.mkdir()
            print(f"{input_dir.name} subdir don't exits! Creating one and moving all mp4 videos inside.")
            for mp4_path in list(session.glob('**/*.MP4')) + list(session.glob('**/*.mp4')):
                out_path = input_dir.joinpath(mp4_path.name)
                shutil.move(mp4_path, out_path)
        
        # create mapping video if don't exist
        mapping_candidates = [
            input_dir.joinpath('mapping.mp4'),
            input_dir.joinpath('mapping.MP4'),
        ]
        has_mapping_video = any(p.exists() or p.is_symlink() for p in mapping_candidates)
        if not has_mapping_video:
            target_mapping_path = input_dir.joinpath('mapping.mp4')
            max_size = -1
            max_path = None
            for mp4_path in _iter_input_mp4_paths(input_dir):
                size = mp4_path.stat().st_size
                if size > max_size:
                    max_size = size
                    max_path = mp4_path
            if max_path is None:
                raise RuntimeError(f'No eligible mp4 found under {input_dir} to use as mapping.mp4')
            shutil.move(max_path, target_mapping_path)
            print(f"raw_videos/mapping.mp4 don't exist! Renaming largest file {max_path.name}.")
        
        # create gripper calibration video if don't exist
        gripper_cal_dir = input_dir.joinpath('gripper_calibration')
        if not gripper_cal_dir.is_dir():
            gripper_cal_dir.mkdir()
            print("raw_videos/gripper_calibration don't exist! Creating one with the first video of each camera serial.")
            
            serial_start_dict = dict()
            serial_path_dict = dict()
            with ExifToolHelper() as et:
                for mp4_path in _iter_input_mp4_paths(input_dir):
                    if mp4_path.name.startswith('map'):
                        continue
                    
                    start_date = mp4_get_start_datetime(str(mp4_path))
                    meta = list(et.get_metadata(str(mp4_path)))[0]
                    cam_serial = meta['QuickTime:CameraSerialNumber']
                    
                    if cam_serial in serial_start_dict:
                        if start_date < serial_start_dict[cam_serial]:
                            serial_start_dict[cam_serial] = start_date
                            serial_path_dict[cam_serial] = mp4_path
                    else:
                        serial_start_dict[cam_serial] = start_date
                        serial_path_dict[cam_serial] = mp4_path
            
            for serial, path in serial_path_dict.items():
                print(f"Selected {path.name} for camera serial {serial}")
                out_path = gripper_cal_dir.joinpath(path.name)
                shutil.move(path, out_path)

        # look for mp4 video in all subdirectories in input_dir
        input_mp4_paths = _iter_input_mp4_paths(input_dir)
        print(f'Found {len(input_mp4_paths)} eligible MP4 videos')

        with ExifToolHelper() as et:
            for mp4_path in input_mp4_paths:
                if mp4_path.is_symlink():
                    print(f"Skipping {mp4_path.name}, already moved.")
                    continue

                start_date = mp4_get_start_datetime(str(mp4_path))
                meta = list(et.get_metadata(str(mp4_path)))[0]
                cam_serial = meta['QuickTime:CameraSerialNumber']
                out_dname = 'demo_' + cam_serial + '_' + start_date.strftime(r"%Y.%m.%d_%H.%M.%S.%f")

                # special folders
                if mp4_path.name.startswith('mapping'):
                    out_dname = "mapping"
                elif mp4_path.name.startswith('gripper_cal') or mp4_path.parent.name.startswith('gripper_cal'):
                    out_dname = "gripper_calibration_" + cam_serial + '_' + start_date.strftime(r"%Y.%m.%d_%H.%M.%S.%f")
                
                # create directory
                this_out_dir = output_dir.joinpath(out_dname)
                this_out_dir.mkdir(parents=True, exist_ok=True)
                
                # move videos
                vfname = 'raw_video.mp4'
                out_video_path = this_out_dir.joinpath(vfname)
                shutil.move(mp4_path, out_video_path)

                # create symlink back from original location
                # relative_to's walk_up argument is not avaliable until python 3.12
                dots = os.path.join(*['..'] * len(mp4_path.parent.relative_to(session).parts))
                rel_path = str(out_video_path.relative_to(session))
                symlink_path = os.path.join(dots, rel_path)
                mp4_path.symlink_to(symlink_path)

# %%
if __name__ == '__main__':
    if len(sys.argv) == 1:
        main.main(['--help'])
    else:
        main()
