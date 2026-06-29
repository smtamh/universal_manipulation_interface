"""
Run the post-scale-calibration SLAM stages with optional translation scaling.

Behavior:
- If no prior SLAM results exist, this runs the full 00->06 pipeline on session/demos.
- If demos/mapping/map_atlas.osa and demo camera_trajectory.csv files already exist
  (for example after run_slam_pre_scale_calibration_pipeline.py), it reuses those results and
  starts from the scaling step instead of rebuilding the map / rerunning SLAM.
- If scaling is requested (any of x/y/z != 1.0), scaling does NOT modify session/demos
  in place. Instead it creates session/demos_scaled, which mirrors the demos/ layout.
- A hidden runtime session is used only to satisfy downstream scripts that expect
  session/demos. Visible scaled outputs live under demos_scaled.
- If x=y=z=1.0, it reuses the original session/demos directly.

Example:
python run_slam_post_scale_calibration_pipeline.py --x 1.0 --y 1.0 --z 1.0 <session_dir>
"""

import sys
import os
import csv
import pathlib
import shutil
import click
import subprocess

ROOT_DIR = os.path.dirname(__file__)
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

SCALE_CAL_PREFIX = 'scale_calibration_'


def scale_trajectory_csv(csv_path: pathlib.Path, scale_x: float, scale_y: float, scale_z: float):
    if not csv_path.is_file():
        return False

    with csv_path.open('r', newline='') as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames
        if fieldnames is None:
            raise RuntimeError(f'Empty csv file: {csv_path}')
        required = {'x', 'y', 'z'}
        if not required.issubset(set(fieldnames)):
            raise RuntimeError(f'Missing x/y/z columns in {csv_path}')
        rows = list(reader)

    for row in rows:
        row['x'] = f"{float(row['x']) * scale_x:.9f}"
        row['y'] = f"{float(row['y']) * scale_y:.9f}"
        row['z'] = f"{float(row['z']) * scale_z:.9f}"

    with csv_path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    return True


def _is_scaled_trajectory(rel_path: pathlib.Path):
    if rel_path.name not in {'camera_trajectory.csv', 'mapping_camera_trajectory.csv'}:
        return False
    if len(rel_path.parts) < 2:
        return False

    parent_name = rel_path.parts[0]
    if parent_name.startswith(SCALE_CAL_PREFIX):
        return False
    if parent_name == 'mapping':
        return True
    return rel_path.name == 'camera_trajectory.csv'


def _replace_symlink(dst_path: pathlib.Path, src_path: pathlib.Path):
    if dst_path.exists() or dst_path.is_symlink():
        dst_path.unlink()
    os.symlink(src_path, dst_path)


def create_scaled_demo_tree(session: pathlib.Path, scale_x: float, scale_y: float, scale_z: float):
    src_demo_dir = session.joinpath('demos')
    dst_demo_dir = session.joinpath('demos_scaled')

    if dst_demo_dir.exists() or dst_demo_dir.is_symlink():
        if dst_demo_dir.is_symlink() or dst_demo_dir.is_file():
            dst_demo_dir.unlink()
        else:
            shutil.rmtree(dst_demo_dir)
    dst_demo_dir.mkdir(parents=True, exist_ok=True)

    scaled_paths = []
    for src_path in sorted(src_demo_dir.rglob('*')):
        rel_path = src_path.relative_to(src_demo_dir)
        dst_path = dst_demo_dir.joinpath(rel_path)

        if src_path.is_dir():
            dst_path.mkdir(parents=True, exist_ok=True)
            continue

        dst_path.parent.mkdir(parents=True, exist_ok=True)
        if _is_scaled_trajectory(rel_path):
            shutil.copy2(src_path, dst_path)
            if scale_trajectory_csv(dst_path, scale_x, scale_y, scale_z):
                scaled_paths.append(dst_path)
        else:
            _replace_symlink(dst_path, src_path)

    print(f'Scaled {len(scaled_paths)} trajectory csv files into {dst_demo_dir} with x={scale_x}, y={scale_y}, z={scale_z}')
    for path in scaled_paths:
        print(path)
    return dst_demo_dir


def create_scaled_runtime(session: pathlib.Path, scaled_demo_dir: pathlib.Path):
    runtime_dir = session.joinpath('.demos_scaled_runtime')
    if runtime_dir.exists() or runtime_dir.is_symlink():
        if runtime_dir.is_symlink() or runtime_dir.is_file():
            runtime_dir.unlink()
        else:
            shutil.rmtree(runtime_dir)
    runtime_dir.mkdir(parents=True, exist_ok=True)
    os.symlink(scaled_demo_dir, runtime_dir.joinpath('demos'))
    return runtime_dir


def has_reusable_slam_outputs(session: pathlib.Path):
    demo_dir = session.joinpath('demos')
    map_path = demo_dir.joinpath('mapping', 'map_atlas.osa')
    if not map_path.is_file():
        return False
    demo_csv_paths = [
        p for p in demo_dir.glob('demo_*/camera_trajectory.csv')
        if p.is_file()
    ]
    return len(demo_csv_paths) > 0


def needs_scaled_workspace(scale_x: float, scale_y: float, scale_z: float):
    return not (scale_x == 1.0 and scale_y == 1.0 and scale_z == 1.0)


@click.command()
@click.argument('session_dir', nargs=-1)
@click.option('-c', '--calibration_dir', type=str, default=None)
@click.option('--x', 'scale_x', type=float, default=1.0, show_default=True, help='Post-SLAM scale multiplier for translation x.')
@click.option('--y', 'scale_y', type=float, default=1.0, show_default=True, help='Post-SLAM scale multiplier for translation y.')
@click.option('--z', 'scale_z', type=float, default=1.0, show_default=True, help='Post-SLAM scale multiplier for translation z.')
@click.option('--force_full_slam', is_flag=True, default=False, help='Ignore existing map/trajectory outputs and rerun 00->03.')
def main(session_dir, calibration_dir, scale_x, scale_y, scale_z, force_full_slam):
    script_dir = pathlib.Path(__file__).parent.joinpath('scripts_slam_pipeline')
    if calibration_dir is None:
        calibration_dir = pathlib.Path(__file__).parent.joinpath('example', 'calibration')
    else:
        calibration_dir = pathlib.Path(calibration_dir)
    assert calibration_dir.is_dir()

    for session in session_dir:
        session = pathlib.Path(os.path.expanduser(session)).absolute()
        demo_dir = session.joinpath('demos')
        mapping_dir = demo_dir.joinpath('mapping')
        map_path = mapping_dir.joinpath('map_atlas.osa')

        reuse_existing = (not force_full_slam) and has_reusable_slam_outputs(session)

        if reuse_existing:
            print('############# Reusing existing shared map and SLAM trajectories ###########')
            print(map_path)
        else:
            print('############## 00_process_videos #############')
            script_path = script_dir.joinpath('00_process_videos.py')
            cmd = ['python', str(script_path), str(session)]
            result = subprocess.run(cmd)
            assert result.returncode == 0

            print('############# 01_extract_gopro_imu ###########')
            script_path = script_dir.joinpath('01_extract_gopro_imu.py')
            cmd = ['python', str(script_path), str(session)]
            result = subprocess.run(cmd)
            assert result.returncode == 0

            print('############# 02_create_map ###########')
            script_path = script_dir.joinpath('02_create_map.py')
            assert mapping_dir.is_dir()
            if not map_path.is_file():
                cmd = [
                    'python', str(script_path),
                    '--input_dir', str(mapping_dir),
                    '--map_path', str(map_path)
                ]
                result = subprocess.run(cmd)
                assert result.returncode == 0
                assert map_path.is_file()

            print('############# 03_batch_slam ###########')
            script_path = script_dir.joinpath('03_batch_slam.py')
            cmd = [
                'python', str(script_path),
                '--input_dir', str(demo_dir),
                '--map_path', str(map_path)
            ]
            result = subprocess.run(cmd)
            assert result.returncode == 0

        dataset_plan_path = session.joinpath('dataset_plan.pkl')
        if needs_scaled_workspace(scale_x, scale_y, scale_z):
            print('############# 03b_create_scaled_workspace ###########')
            working_demo_dir = create_scaled_demo_tree(session, scale_x, scale_y, scale_z)
            runtime_session = create_scaled_runtime(session, working_demo_dir)
            calib_session = runtime_session
            dataset_plan_path = working_demo_dir.joinpath('dataset_plan.pkl')
        else:
            print('############# 03b_no_scaling_requested ###########')
            print('Using original demos/session because x=y=z=1.0')
            working_demo_dir = demo_dir
            runtime_session = session
            calib_session = session

        print('############# 04_detect_aruco ###########')
        script_path = script_dir.joinpath('04_detect_aruco.py')
        camera_intrinsics = calibration_dir.joinpath('gopro_intrinsics_2_7k.json')
        aruco_config = calibration_dir.joinpath('aruco_config.yaml')
        assert camera_intrinsics.is_file()
        assert aruco_config.is_file()

        cmd = [
            'python', str(script_path),
            '--input_dir', str(working_demo_dir),
            '--camera_intrinsics', str(camera_intrinsics),
            '--aruco_yaml', str(aruco_config)
        ]
        result = subprocess.run(cmd)
        assert result.returncode == 0

        print('############# 05_run_calibrations ###########')
        script_path = script_dir.joinpath('05_run_calibrations.py')
        cmd = ['python', str(script_path), str(calib_session)]
        result = subprocess.run(cmd)
        assert result.returncode == 0

        print('############# 06_generate_dataset_plan ###########')
        script_path = script_dir.joinpath('06_generate_dataset_plan.py')
        cmd = ['python', str(script_path), '--input', str(runtime_session), '--output', str(dataset_plan_path)]
        result = subprocess.run(cmd)
        assert result.returncode == 0

        print('############# Output workspace ###########')
        print(f'Demo workspace: {working_demo_dir}')
        print(f'Dataset plan: {dataset_plan_path}')


if __name__ == '__main__':
    main()
