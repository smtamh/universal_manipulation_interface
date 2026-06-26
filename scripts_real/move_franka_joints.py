import sys
import re
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR))

import click
import yaml
import numpy as np

from umi.real_world.franka_interpolation_controller import FrankaInterface


def parse_joint_value(value: str) -> float:
    """
    Accepts:
        0
        1.57
        pi/2
        -pi/2
        3*pi/4
        -3*pi/4
        3pi/4
        -3pi/4
        0.5pi
    """
    expr = value.strip().lower()
    expr = expr.replace("π", "pi")

    # Allow numbers, pi, arithmetic operators, parentheses, and scientific notation.
    if not re.fullmatch(r"[0-9eE+\-*/().\s pi]+", expr):
        raise click.BadParameter(
            f'Invalid joint value "{value}". Use a float or expressions like -pi/2, 3*pi/4, or 3pi/4.'
        )

    # Convert implicit multiplication:
    # 3pi     -> 3*pi
    # 0.5pi   -> 0.5*pi
    # (1+2)pi -> (1+2)*pi
    expr = re.sub(r"(?<=[0-9)])\s*(?=pi\b)", "*", expr)

    # pi2 -> pi*2, pi( ... ) -> pi*( ... )
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


def load_robot_ip(robot_config: str) -> str:
    robot_config_path = Path(robot_config).expanduser()
    with robot_config_path.open("r") as f:
        robot_config_data = yaml.safe_load(f)

    robots_config = robot_config_data["robots"]
    return robots_config[0]["robot_ip"]


@click.command()
@click.option(
    "--robot_config", "-rc",
    default="example/eval_robots_config.yaml",
    show_default=True,
    help="Path to robot_config yaml file. Used to get robot_ip if --robot_hostname is not given."
)
@click.option(
    "-rh", "--robot_hostname",
    default=None,
    help="Override robot IP or hostname. If not given, robot_ip is loaded from --robot_config."
)
@click.option(
    "-rp", "--robot_port",
    type=int,
    default=4242,
    show_default=True,
    help="Port of the Franka interface server."
)
@click.option(
    "-j", "--joint",
    "joints",
    type=str,
    nargs=7,
    required=True,
    help="Seven target joint values in radians. Example: --joint 0 0 0 -3pi/4 0 pi/2 0"
)
@click.option(
    "-t", "--time-to-go",
    type=float,
    default=3.0,
    show_default=True,
    help="Motion duration in seconds."
)
@click.option(
    "-y", "--yes",
    is_flag=True,
    default=False,
    help="Skip the confirmation prompt."
)
def main(robot_config, robot_hostname, robot_port, joints, time_to_go, yes):
    if time_to_go <= 0:
        raise click.UsageError("--time-to-go must be positive.")

    if robot_hostname is None:
        robot_hostname = load_robot_ip(robot_config)

    target = np.asarray([parse_joint_value(x) for x in joints], dtype=np.float64)

    click.echo(f"Using robot IP:   {robot_hostname}")
    click.echo(f"Using robot port: {robot_port}")

    robot = FrankaInterface(ip=robot_hostname, port=robot_port)
    try:
        current = robot.get_joint_positions()

        click.echo(f"Current joints: {np.array2string(current, precision=6)}")
        click.echo(f"Target joints:  {np.array2string(target, precision=6)}")
        click.echo(f"Time to go:     {time_to_go:.3f}s")

        if not yes:
            click.confirm("Send the Franka to this joint target?", abort=True)

        robot.move_to_joint_positions(target, time_to_go)
        click.echo("Joint command sent.")

    finally:
        robot.close()


if __name__ == "__main__":
    main()