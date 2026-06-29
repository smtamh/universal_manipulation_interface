"""
Replay a UMI dataset episode on the real single-arm Franka UMI setup.

Controls:
- Click the OpenCV window to focus it.
- Press "m" to prepare the selected episode from the current robot TCP pose.
- Press "c" to start replay.
- Press "s" to stop replay.
- Press "e" / "w" to move to the next / previous episode.
- Press "r" to reset the Panda joints.
- Press "q" to quit.
"""
# %%
import os
import re
import time
from multiprocessing.managers import SharedMemoryManager

import click
import cv2
import numpy as np
import yaml
import zarr

from diffusion_policy.codecs.imagecodecs_numcodecs import register_codecs
from diffusion_policy.common.replay_buffer import ReplayBuffer
from umi.common.interpolation_util import PoseInterpolator, get_interp1d
from umi.common.pose_util import pose_to_mat, mat_to_pose
from umi.common.precise_sleep import precise_wait
from umi.real_world.umi_env import UmiEnv

register_codecs()

DEFAULT_HOME_JOINTS = np.array(
    [-np.pi / 16.0, 0.0, 0.0, -3.0 * np.pi / 4.0, 0.0, 3.0 * np.pi / 4.0, 3.0 * np.pi / 16.0],
    dtype=np.float64,
)
RESET_DURATION = 6.0
DEFAULT_DATA_FREQUENCY = 59.94


def parse_joint_value(value: str) -> float:
    expr = value.strip().lower().replace("π", "pi")
    if not re.fullmatch(r"[0-9eE+\-*/().\s pi]+", expr):
        raise click.BadParameter(
            f'Invalid joint value "{value}". Use a float or expressions like -pi/2, 3*pi/4, or 3pi/4.'
        )
    expr = re.sub(r"(?<=[0-9)])\s*(?=pi\b)", "*", expr)
    expr = re.sub(r"(?<=pi)\s*(?=[0-9(])", "*", expr)
    try:
        result = float(eval(expr, {"__builtins__": {}}, {"pi": np.pi}))
    except Exception as exc:
        raise click.BadParameter(
            f'Invalid joint value "{value}". Use a float or expressions like -pi/2, 3*pi/4, or 3pi/4.'
        ) from exc
    if not np.isfinite(result):
        raise click.BadParameter(f'Invalid joint value "{value}". Result is not finite.')
    return result


def wait_for_fresh_robot_state(env, min_timestamp, timeout=5.0, poll_interval=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = env.get_robot_state()
        recv_ts = float(state.get('robot_receive_timestamp', 0.0))
        if recv_ts >= min_timestamp:
            return state
        time.sleep(poll_interval)
    raise RuntimeError(f'Failed to receive fresh robot state after reset within {timeout:.1f}s')


def reset_to_home(env, home_joints, gripper_width, duration):
    target_time = time.time() + duration
    env.robot.moveJ(home_joints, duration=duration)
    env.gripper.schedule_waypoint(gripper_width, target_time=target_time)
    print('Reset Panda to home joints.', flush=True)
    time.sleep(duration)
    state = wait_for_fresh_robot_state(env, min_timestamp=target_time)
    print('Reset Panda to home joints finished.', flush=True)
    return state


def build_replay_targets(episode_data, replay_frequency: float, data_frequency: float, replay_start_pose: np.ndarray, time_scale: float):
    data_pose = np.concatenate(
        [episode_data['robot0_eef_pos'], episode_data['robot0_eef_rot_axis_angle']],
        axis=-1,
    )
    n_data_samples = len(data_pose)
    data_timestamps = np.arange(n_data_samples, dtype=np.float32) / data_frequency
    data_pose_interpolator = PoseInterpolator(data_timestamps, data_pose)
    data_gripper_interpolator = get_interp1d(
        data_timestamps + 0.05,
        episode_data['robot0_gripper_width'],
    )

    replay_duration = data_timestamps[-1] * time_scale
    exec_timestamps = np.arange(int(np.floor(replay_duration * replay_frequency)) + 1) / replay_frequency
    dataset_exec_timestamps = np.clip(exec_timestamps / time_scale, 0, data_timestamps[-1])
    exec_data_idxs = np.round(dataset_exec_timestamps * data_frequency).astype(np.int32)

    dataset_start_pose_mat = pose_to_mat(data_pose_interpolator(0.0))
    replay_start_pose_mat = pose_to_mat(replay_start_pose)

    actions = []
    for dataset_t in dataset_exec_timestamps:
        dataset_pose_mat = pose_to_mat(data_pose_interpolator(dataset_t))
        rel_pose_mat = np.linalg.inv(dataset_start_pose_mat) @ dataset_pose_mat
        target_pose = mat_to_pose(replay_start_pose_mat @ rel_pose_mat)
        target_grip = float(data_gripper_interpolator(dataset_t))
        actions.append(np.concatenate([target_pose, [target_grip]], axis=-1))

    return np.asarray(actions, dtype=np.float32), exec_timestamps, exec_data_idxs


def draw_text(img, text):
    cv2.putText(
        img,
        text,
        (10, 20),
        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
        fontScale=0.5,
        lineType=cv2.LINE_AA,
        thickness=3,
        color=(0, 0, 0),
    )
    cv2.putText(
        img,
        text,
        (10, 20),
        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
        fontScale=0.5,
        thickness=1,
        color=(255, 255, 255),
    )
    return img


@click.command()
@click.option('--input', '-i', required=True, help='Path to dataset.zarr.zip')
@click.option('--output', '-o', required=True, help='Directory to save recording')
@click.option('--robot_config', '-rc', required=True, help='Path to robot_config yaml file')
@click.option('--replay_episode', '-re', type=int, default=0)
@click.option('--camera_reorder', '-cr', default='0')
@click.option('--vis_camera_idx', default=0, type=int, help='Which camera to visualize.')
@click.option('--joint', 'home_joints', type=str, nargs=7, default=None, help='Seven home joint values in radians. Example: --joint -pi/16 -pi/8 0 -7pi/8 0 3pi/4 3pi/16')
@click.option('--frequency', '-f', default=60.0, type=float, help='Replay control frequency in Hz.')
@click.option('--data_frequency', default=DEFAULT_DATA_FREQUENCY, type=float, help='Dataset sample frequency in Hz.')
@click.option('--time_scale', default=1.0, type=float, help='Replay time scale. Values >1 execute the dataset trajectory slower.')
@click.option('--command_latency', '-cl', default=0.01, type=float, help='Latency between command creation and robot execution in seconds.')
def main(input, output, robot_config, replay_episode, camera_reorder, vis_camera_idx,
         home_joints, frequency, data_frequency, time_scale, command_latency):
    max_gripper_width = 0.09
    if time_scale <= 0:
        raise click.BadParameter('--time_scale must be positive.')
    if home_joints is None:
        home_joints = DEFAULT_HOME_JOINTS.copy()
    else:
        home_joints = np.asarray([parse_joint_value(x) for x in home_joints], dtype=np.float64)

    robot_config_data = yaml.safe_load(open(os.path.expanduser(robot_config), 'r'))
    robots_config = robot_config_data['robots']
    grippers_config = robot_config_data['grippers']
    robot_config = robots_config[0]
    gripper_config = grippers_config[0]

    # load replay buffer
    with zarr.ZipStore(input, mode='r') as zip_store:
        replay_buffer = ReplayBuffer.copy_from_store(src_store=zip_store, store=zarr.MemoryStore())
    obs_res = replay_buffer['camera0_rgb'].shape[1:-1][::-1]

    # setup experiment
    dt = 1.0 / frequency
    with SharedMemoryManager() as shm_manager:
        with UmiEnv(
            output_dir=output,
            robot_ip=robot_config['robot_ip'],
            gripper_ip=gripper_config['gripper_ip'],
            gripper_port=gripper_config.get('gripper_port', 1000),
            robot_type=robot_config.get('robot_type', 'franka'),
            gripper_type=gripper_config.get('gripper_type', 'dyros'),
            frequency=frequency,
            obs_image_resolution=obs_res,
            obs_float32=True,
            camera_reorder=[int(x) for x in camera_reorder],
            init_joints=False,
            enable_multi_cam_vis=True,
            # latency
            camera_obs_latency=0.125,
            robot_obs_latency=robot_config.get('robot_obs_latency', 0.0001),
            gripper_obs_latency=gripper_config.get('gripper_obs_latency', 0.01),
            robot_action_latency=0.0,
            gripper_action_latency=0.0,
            # action
            max_pos_speed=2.0,
            max_rot_speed=6.0,
            shm_manager=shm_manager) as env:
            cv2.setNumThreads(2)
            print('Waiting for camera')
            time.sleep(1.0)

            state = reset_to_home(
                env=env,
                home_joints=home_joints,
                gripper_width=max_gripper_width,
                duration=RESET_DURATION,
            )

            episode_data = None
            replay_start_pose = None
            print('Ready!')
            while True:
                # ========= human control loop ==========
                print('Human in control!')
                state = env.get_robot_state()
                target_pose = np.array(state['ActualTCPPose'], dtype=np.float64, copy=True)
                gripper_target_pos = max_gripper_width
                t_start = time.monotonic()
                iter_idx = 0
                while True:
                    # calculate timing
                    t_cycle_end = t_start + (iter_idx + 1) * dt
                    t_sample = t_cycle_end - command_latency
                    t_command_target = t_cycle_end + dt

                    # pump obs
                    obs = env.get_obs()

                    # visualize
                    episode_slice = replay_buffer.get_episode_slice(replay_episode)
                    start_idx = episode_slice.start
                    match_img = replay_buffer['camera0_rgb'][start_idx]
                    start_gripper_width = replay_buffer['robot0_gripper_width'][start_idx]

                    vis_img = obs[f'camera{vis_camera_idx}_rgb'][-1]
                    match_img = match_img.astype(np.float32) / 255.0
                    avg_img = (vis_img + match_img) / 2.0
                    vis_img = np.concatenate([vis_img, avg_img, match_img], axis=1)
                    vis_img = draw_text(vis_img, f'Episode: {replay_episode}')

                    cv2.imshow('default', vis_img[..., ::-1])
                    key_stroke = cv2.pollKey()
                    if key_stroke == ord('q'):
                        # Exit program
                        env.end_episode()
                        return
                    elif key_stroke == ord('c'):
                        # Exit human control loop
                        if episode_data is not None:
                            # hand control over to replay
                            break
                        print('Episode is not prepared yet. Press "m" first.')
                    elif key_stroke == ord('e'):
                        # Next episode
                        replay_episode = min(replay_episode + 1, replay_buffer.n_episodes - 1)
                        episode_data = None
                        replay_start_pose = None
                    elif key_stroke == ord('w'):
                        # Prev episode
                        replay_episode = max(replay_episode - 1, 0)
                        episode_data = None
                        replay_start_pose = None
                    elif key_stroke == ord('r'):
                        state = reset_to_home(
                            env=env,
                            home_joints=home_joints,
                            gripper_width=max_gripper_width,
                            duration=RESET_DURATION,
                        )
                        target_pose = np.array(state['ActualTCPPose'], dtype=np.float64, copy=True)
                        gripper_target_pos = max_gripper_width
                        episode_data = None
                        replay_start_pose = None
                        t_start = time.monotonic()
                        iter_idx = 0
                        continue
                    elif key_stroke == ord('m'):
                        # prepare episode for replay from the current robot pose
                        state = env.get_robot_state()
                        gripper_target_pos = float(start_gripper_width)
                        episode_data = replay_buffer.get_episode(replay_episode)
                        replay_start_pose = np.array(state['ActualTCPPose'], dtype=np.float64, copy=True)
                        target_pose = replay_start_pose.copy()
                        print(f'Prepared episode {replay_episode} for replay from the current robot pose.', flush=True)

                    precise_wait(t_sample)
                    #################################### get teleop command #############################################
                    # SpaceMouse teleop is intentionally disabled for this replay entrypoint. The loop keeps
                    # streaming the current target pose/gripper width so the real robot remains under command
                    # until an episode is prepared and started.
                    #######################################################################################################

                    action = np.zeros((7,), dtype=np.float64)
                    action[:6] = target_pose
                    action[-1] = gripper_target_pos

                    # execute hold command
                    if t_command_target > (time.monotonic() + 0.001):
                        env.exec_actions(
                            actions=[action],
                            timestamps=[t_command_target - time.monotonic() + time.time()],
                        )
                    precise_wait(t_cycle_end)
                    iter_idx += 1

                # ========== replay control loop ==============
                try:
                    actions, exec_timestamps, exec_data_idxs = build_replay_targets(
                        episode_data=episode_data,
                        replay_frequency=frequency,
                        data_frequency=data_frequency,
                        replay_start_pose=replay_start_pose,
                        time_scale=time_scale,
                    )

                    # start episode
                    start_delay = 1.0
                    eval_t_start = time.time() + start_delay
                    t_start = time.monotonic() + start_delay
                    env.start_episode(eval_t_start)
                    # wait for 1/30 sec to get the closest frame actually
                    # reduces overall latency
                    frame_latency = 1 / 60
                    precise_wait(eval_t_start - frame_latency, time_func=time.time)
                    print('Started!')

                    for iter_idx, t in enumerate(exec_timestamps):
                        t_cycle_start = t_start + t
                        t_cycle_end = t_cycle_start + dt
                        data_idx = exec_data_idxs[iter_idx]
                        action = actions[iter_idx]

                        obs = env.get_obs()
                        env.exec_actions(
                            actions=[action],
                            timestamps=[t_cycle_end - time.monotonic() + time.time()],
                        )

                        # plot image overlay
                        img = episode_data['camera0_rgb'][data_idx]
                        vis_img = obs[f'camera{vis_camera_idx}_rgb'][-1]
                        match_img = img.astype(np.float32) / 255.0
                        avg_img = (vis_img + match_img) / 2.0
                        vis_img = np.concatenate([vis_img, avg_img, match_img], axis=1)
                        cv2.imshow('default', vis_img[..., ::-1])

                        key_stroke = cv2.pollKey()
                        if key_stroke == ord('s'):
                            # Stop episode
                            # Hand control back to human
                            print('Stopped.')
                            break

                        precise_wait(t_cycle_end)
                    env.end_episode()
                    state = env.get_robot_state()

                except KeyboardInterrupt:
                    print('Interrupted!')
                    env.end_episode()

                print('Stopped.')


# %%
if __name__ == '__main__':
    main()