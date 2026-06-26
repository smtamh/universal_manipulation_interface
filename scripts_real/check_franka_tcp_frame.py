import sys
import time
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR))

import click
import numpy as np
import yaml

from umi.common.pose_util import pose_to_mat
from umi.real_world.franka_interpolation_controller import FrankaInterface


def format_vec(vec: np.ndarray) -> str:
    return "[" + ", ".join(f"{x: .5f}" for x in vec.tolist()) + "]"


def load_robot_ip(robot_config: str) -> str:
    robot_config_path = Path(robot_config).expanduser()
    with robot_config_path.open("r") as f:
        robot_config_data = yaml.safe_load(f)
    return robot_config_data["robots"][0]["robot_ip"]


@click.command()
@click.option(
    "--robot_config", "-rc",
    default="example/eval_robots_config.yaml",
    show_default=True,
    help="Path to robot config YAML. Used when --robot_hostname is not given."
)
@click.option(
    "-rh", "--robot_hostname",
    default=None,
    help="Override Franka interface server host. If omitted, robot_ip is loaded from --robot_config."
)
@click.option(
    "-rp", "--robot_port",
    default=4242,
    show_default=True,
    type=int,
    help="Port of the Franka interface server."
)
@click.option(
    "--frequency", "-f",
    default=2.0,
    show_default=True,
    type=float,
    help="Polling frequency in Hz."
)
@click.option(
    "--count", "-n",
    default=1,
    show_default=True,
    type=int,
    help="Number of samples to print. Use 0 or a negative value to stream until Ctrl-C."
)
def main(robot_config, robot_hostname, robot_port, frequency, count):
    if frequency <= 0:
        raise click.UsageError("--frequency must be positive.")
    if robot_hostname is None:
        robot_hostname = load_robot_ip(robot_config)

    dt = 1.0 / frequency
    robot = FrankaInterface(ip=robot_hostname, port=robot_port)

    click.echo(f"Using robot host: {robot_hostname}")
    click.echo(f"Using robot port: {robot_port}")
    click.echo("Reading Franka TCP pose in the robot base frame.")
    click.echo("Pose format: [x, y, z, rx, ry, rz], with rotation as axis-angle.")

    sample_idx = 0
    try:
        while True:
            pose = np.asarray(robot.get_ee_pose(), dtype=np.float64)
            pose_mat = pose_to_mat(pose)
            x_axis = pose_mat[:3, 0]
            y_axis = pose_mat[:3, 1]
            z_axis = pose_mat[:3, 2]

            click.echo(f"\nSample {sample_idx}")
            click.echo(f"ActualTCPPose: {format_vec(pose)}")
            click.echo(f"position_xyz_m: {format_vec(pose[:3])}")
            click.echo(f"rotvec_xyz_rad: {format_vec(pose[3:])}")
            click.echo(f"tcp_x_axis_in_base: {format_vec(x_axis)}")
            click.echo(f"tcp_y_axis_in_base: {format_vec(y_axis)}")
            click.echo(f"tcp_z_axis_in_base: {format_vec(z_axis)}")

            sample_idx += 1
            if count > 0 and sample_idx >= count:
                break
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        robot.close()


if __name__ == "__main__":
    main()
