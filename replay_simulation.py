"""
Replay a UMI dataset episode on a simulated Franka Panda in PyBullet.

Controls:
- Click the OpenCV window to focus it.
- Press "m" to prepare the selected episode from the current simulated EE pose.
- Press "c" to start replay.
- Press "s" to stop replay.
- Press "e" / "w" to move to the next / previous episode.
- Press "r" to reset the Panda joints.
- Press "q" to quit.
"""
# %%
import sys
import os

ROOT_DIR = os.path.dirname(__file__)
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

# %%
import time

import click
import cv2
import numpy as np
import scipy.spatial.transform as st
import zarr

from diffusion_policy.common.replay_buffer import ReplayBuffer
from diffusion_policy.codecs.imagecodecs_numcodecs import register_codecs
from umi.common.interpolation_util import PoseInterpolator, get_interp1d
from umi.common.pose_util import pose_to_mat, mat_to_pose

register_codecs()

try:
    import pybullet as p
    import pybullet_data
except ImportError as exc:
    p = None
    pybullet_data = None
    _PYBULLET_IMPORT_ERROR = exc
else:
    _PYBULLET_IMPORT_ERROR = None


ARM_JOINT_NAMES = (
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
)
FINGER_JOINT_NAMES = ("panda_finger_joint1", "panda_finger_joint2")
FLANGE_LINK_CANDIDATES = ("panda_link8", "panda_hand")
DEFAULT_HOME_JOINTS = np.array(
    [-np.pi / 16.0, 0.0, 0.0, -3.0 * np.pi / 4.0, 0.0, 3.0 * np.pi / 4.0, 3.0 * np.pi / 16.0],
    dtype=np.float64,
)

# Match the real Franka controller TCP convention by reading and commanding
# a tip frame derived from the flange with a fixed rigid transform.
tx_flangerot90_tip = np.identity(4)
tx_flangerot90_tip[:3, 3] = np.array([-0.0336, 0.0, 0.247], dtype=np.float64)

tx_flangerot45_flangerot90 = np.identity(4)
tx_flangerot45_flangerot90[:3, :3] = st.Rotation.from_euler('x', [np.pi / 2.0]).as_matrix()

tx_flange_flangerot45 = np.identity(4)
tx_flange_flangerot45[:3, :3] = st.Rotation.from_euler('z', [np.pi / 4.0]).as_matrix()

tx_flange_tip = tx_flange_flangerot45 @ tx_flangerot45_flangerot90 @ tx_flangerot90_tip
tx_tip_flange = np.linalg.inv(tx_flange_tip)


class PandaSimulation:
    def __init__(self, gui: bool = True, sim_frequency: float = 240.0):
        if _PYBULLET_IMPORT_ERROR is not None:
            raise click.ClickException(
                "PyBullet is required for simulation replay. "
                "Install `pybullet` and `pybullet_data` in your environment."
            ) from _PYBULLET_IMPORT_ERROR

        connection_mode = p.GUI if gui else p.DIRECT
        self.client_id = p.connect(connection_mode)
        if self.client_id < 0:
            raise click.ClickException("Failed to connect to PyBullet.")

        self.sim_frequency = sim_frequency
        self.time_step = 1.0 / sim_frequency
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.time_step)
        p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)

        self.arm_joint_indices = []
        self.finger_joint_indices = []
        self.flange_link_index = None
        self._build_joint_index_maps()
        self.reset(DEFAULT_HOME_JOINTS)

    def _build_joint_index_maps(self):
        name_to_joint_index = {}
        link_name_to_index = {}
        for joint_idx in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, joint_idx)
            joint_name = joint_info[1].decode("utf-8")
            link_name = joint_info[12].decode("utf-8")
            name_to_joint_index[joint_name] = joint_idx
            link_name_to_index[link_name] = joint_idx

        self.arm_joint_indices = [name_to_joint_index[name] for name in ARM_JOINT_NAMES]
        self.finger_joint_indices = [name_to_joint_index[name] for name in FINGER_JOINT_NAMES]

        for link_name in FLANGE_LINK_CANDIDATES:
            if link_name in link_name_to_index:
                self.flange_link_index = link_name_to_index[link_name]
                break
        if self.flange_link_index is None:
            raise click.ClickException(
                f"Could not find a Franka flange-equivalent link. Tried {FLANGE_LINK_CANDIDATES}."
            )

    def reset(self, joints: np.ndarray):
        for joint_idx, joint_value in zip(self.arm_joint_indices, joints):
            p.resetJointState(self.robot_id, joint_idx, float(joint_value))
            p.setJointMotorControl2(
                self.robot_id,
                joint_idx,
                p.POSITION_CONTROL,
                targetPosition=float(joint_value),
                force=200.0,
            )
        self.set_gripper_width(0.08)
        self.step(20)

    def get_ee_pose(self) -> np.ndarray:
        link_state = p.getLinkState(self.robot_id, self.flange_link_index, computeForwardKinematics=True)
        flange_pos = np.asarray(link_state[4], dtype=np.float64)
        flange_quat_xyzw = np.asarray(link_state[5], dtype=np.float64)
        flange_pose = np.concatenate(
            [flange_pos, st.Rotation.from_quat(flange_quat_xyzw).as_rotvec()],
            axis=-1,
        )
        return mat_to_pose(pose_to_mat(flange_pose) @ tx_flange_tip)

    def command_pose(self, pose: np.ndarray, gripper_width: float):
        flange_pose = mat_to_pose(pose_to_mat(pose) @ tx_tip_flange)
        quat_xyzw = st.Rotation.from_rotvec(flange_pose[3:]).as_quat()
        joint_targets = p.calculateInverseKinematics(
            self.robot_id,
            self.flange_link_index,
            targetPosition=flange_pose[:3].tolist(),
            targetOrientation=quat_xyzw.tolist(),
            maxNumIterations=200,
            residualThreshold=1e-5,
        )
        for joint_idx, joint_target in zip(self.arm_joint_indices, joint_targets[: len(self.arm_joint_indices)]):
            p.setJointMotorControl2(
                self.robot_id,
                joint_idx,
                p.POSITION_CONTROL,
                targetPosition=float(joint_target),
                force=200.0,
                positionGain=0.2,
                velocityGain=1.0,
            )
        self.set_gripper_width(gripper_width)

    def set_gripper_width(self, width: float):
        clamped_width = float(np.clip(width, 0.0, 0.08))
        finger_target = clamped_width / 2.0
        for joint_idx in self.finger_joint_indices:
            p.setJointMotorControl2(
                self.robot_id,
                joint_idx,
                p.POSITION_CONTROL,
                targetPosition=finger_target,
                force=40.0,
            )

    def step(self, n_steps: int = 1):
        for _ in range(n_steps):
            p.stepSimulation()

    def close(self):
        if p is not None and p.isConnected(self.client_id):
            p.disconnect(self.client_id)


def build_sim_targets(episode_data, replay_frequency: float, data_frequency: float, sim_start_pose: np.ndarray, time_scale: float):
    data_pose = np.concatenate(
        [episode_data["robot0_eef_pos"], episode_data["robot0_eef_rot_axis_angle"]],
        axis=-1,
    )
    n_data_samples = len(data_pose)
    data_timestamps = np.arange(n_data_samples, dtype=np.float32) / data_frequency
    data_pose_interpolator = PoseInterpolator(data_timestamps, data_pose)
    data_gripper_interpolator = get_interp1d(
        data_timestamps + 0.05,
        episode_data["robot0_gripper_width"],
    )

    replay_duration = data_timestamps[-1] * time_scale
    exec_timestamps = np.arange(int(np.floor(replay_duration * replay_frequency)) + 1) / replay_frequency
    dataset_exec_timestamps = np.clip(exec_timestamps / time_scale, 0, data_timestamps[-1])
    exec_data_idxs = np.round(dataset_exec_timestamps * data_frequency).astype(np.int32)

    dataset_start_pose = data_pose_interpolator(0.0)
    dataset_start_pose_mat = pose_to_mat(dataset_start_pose)
    sim_start_pose_mat = pose_to_mat(sim_start_pose)

    actions = []
    for dataset_t in dataset_exec_timestamps:
        dataset_pose = data_pose_interpolator(dataset_t)
        dataset_pose_mat = pose_to_mat(dataset_pose)
        rel_pose_mat = np.linalg.inv(dataset_start_pose_mat) @ dataset_pose_mat
        sim_target_pose_mat = sim_start_pose_mat @ rel_pose_mat
        sim_target_pose = mat_to_pose(sim_target_pose_mat)
        sim_target_grip = float(data_gripper_interpolator(dataset_t))
        actions.append(np.concatenate([sim_target_pose, [sim_target_grip]], axis=-1))

    return np.asarray(actions, dtype=np.float32), exec_timestamps, exec_data_idxs


def render_dataset_frame(img: np.ndarray, episode_idx: int, prepared: bool, replaying: bool):
    vis_img = img.astype(np.float32) / 255.0
    text = f"Episode: {episode_idx} | prepared={prepared} | replaying={replaying}"
    cv2.putText(
        vis_img,
        text,
        (10, 20),
        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
        fontScale=0.5,
        thickness=1,
        color=(1.0, 1.0, 1.0),
    )
    cv2.imshow("dataset_replay", vis_img[..., ::-1])


@click.command()
@click.option("--input", "-i", required=True, help="Path to dataset.zarr.zip")
@click.option("--replay_episode", "-re", type=int, default=0)
@click.option("--frequency", "-f", default=60.0, type=float, help="Replay control frequency in Hz.")
@click.option("--data_frequency", default=59.94, type=float, help="Dataset sample frequency in Hz.")
@click.option("--sim_frequency", default=240.0, type=float, help="PyBullet stepping frequency in Hz.")
@click.option("--time_scale", default=1.0, type=float, help="Replay time scale. Values >1 execute the dataset trajectory slower.")
@click.option("--no-gui", is_flag=True, default=False, help="Run PyBullet without GUI.")
def main(input, replay_episode, frequency, data_frequency, sim_frequency, time_scale, no_gui):
    if time_scale <= 0:
        raise click.BadParameter('--time_scale must be positive.')
    with zarr.ZipStore(input, mode="r") as zip_store:
        replay_buffer = ReplayBuffer.copy_from_store(
            src_store=zip_store,
            store=zarr.MemoryStore(),
        )

    sim = PandaSimulation(gui=not no_gui, sim_frequency=sim_frequency)
    cv2.setNumThreads(2)

    sim_steps_per_control = max(1, int(round(sim_frequency / frequency)))
    control_dt = 1.0 / frequency
    prepared_actions = None
    prepared_exec_data_idxs = None
    episode_data = None
    replaying = False
    replay_idx = 0
    replay_start_time = None

    try:
        while True:
            if episode_data is None:
                episode_data = replay_buffer.get_episode(replay_episode)

            frame_idx = 0
            if prepared_exec_data_idxs is not None and len(prepared_exec_data_idxs) > 0:
                frame_idx = int(prepared_exec_data_idxs[min(replay_idx, len(prepared_exec_data_idxs) - 1)])
            img = episode_data["camera0_rgb"][frame_idx]
            render_dataset_frame(
                img=img,
                episode_idx=replay_episode,
                prepared=prepared_actions is not None,
                replaying=replaying,
            )

            key_stroke = cv2.waitKey(1) & 0xFF
            if key_stroke == ord("q"):
                break
            elif key_stroke == ord("e"):
                replay_episode = min(replay_episode + 1, replay_buffer.n_episodes - 1)
                episode_data = replay_buffer.get_episode(replay_episode)
                prepared_actions = None
                prepared_exec_data_idxs = None
                replaying = False
                replay_idx = 0
                print(f"Selected episode {replay_episode}")
            elif key_stroke == ord("w"):
                replay_episode = max(replay_episode - 1, 0)
                episode_data = replay_buffer.get_episode(replay_episode)
                prepared_actions = None
                prepared_exec_data_idxs = None
                replaying = False
                replay_idx = 0
                print(f"Selected episode {replay_episode}")
            elif key_stroke == ord("r"):
                sim.reset(DEFAULT_HOME_JOINTS)
                prepared_actions = None
                prepared_exec_data_idxs = None
                replaying = False
                replay_idx = 0
                print("Reset Panda to home joints.")
            elif key_stroke == ord("m"):
                sim_start_pose = sim.get_ee_pose()
                prepared_actions, _, prepared_exec_data_idxs = build_sim_targets(
                    episode_data=episode_data,
                    replay_frequency=frequency,
                    data_frequency=data_frequency,
                    sim_start_pose=sim_start_pose,
                    time_scale=time_scale,
                )
                replaying = False
                replay_idx = 0
                print(
                    f"Prepared episode {replay_episode} from current simulated EE pose.",
                    flush=True,
                )
            elif key_stroke == ord("c"):
                if prepared_actions is None:
                    print('Episode is not prepared yet. Press "m" first.')
                else:
                    replaying = True
                    replay_idx = 0
                    replay_start_time = time.monotonic() + 1.0
                    print("Started simulation replay.", flush=True)
            elif key_stroke == ord("s"):
                if replaying:
                    replaying = False
                    print("Stopped simulation replay.", flush=True)

            if replaying and prepared_actions is not None:
                now = time.monotonic()
                if now >= replay_start_time + replay_idx * control_dt:
                    if replay_idx < len(prepared_actions):
                        action = prepared_actions[replay_idx]
                        sim.command_pose(action[:6], action[6])
                        replay_idx += 1
                    else:
                        replaying = False
                        print("Finished simulation replay.", flush=True)

            sim.step(sim_steps_per_control)
            time.sleep(sim.time_step * sim_steps_per_control)

    finally:
        sim.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()