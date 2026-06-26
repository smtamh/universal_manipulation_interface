import sys
from pathlib import Path
import time

ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR))

import click

from umi.real_world.dyros_binary_driver import DYROSBinaryDriver, GRIPPER_MAX_WIDTH


def convert_width_to_mm(width: float, unit: str) -> float:
    if unit == "m":
        return width * 1000.0
    if unit == "mm":
        return width
    raise click.UsageError(f"Unsupported unit: {unit}")


def validate_width_mm(width_mm: float) -> None:
    if width_mm < 0.0 or width_mm > GRIPPER_MAX_WIDTH:
        raise click.UsageError(
            f"Target width must be in [0, {GRIPPER_MAX_WIDTH}] mm "
            f"(or [0, {GRIPPER_MAX_WIDTH / 1000.0:.3f}] m)."
        )


@click.command()
@click.option(
    "-w", "--width",
    type=float,
    required=True,
    help="Target UMI gripper width."
)
@click.option(
    "-u", "--unit",
    type=click.Choice(["mm", "m"], case_sensitive=False),
    default="m",
    show_default=True,
    help="Unit for --width."
)
@click.option(
    "-v", "--velocity",
    type=float,
    default=50.0,
    show_default=True,
    help="Command velocity passed to the DYROS driver."
)
@click.option(
    "--wait/--no-wait",
    default=True,
    show_default=True,
    help="Wait until the measured width is within tolerance before exiting."
)
@click.option(
    "--timeout",
    type=float,
    default=5.0,
    show_default=True,
    help="Maximum wait time in seconds."
)
@click.option(
    "--tolerance-mm",
    type=float,
    default=2.0,
    show_default=True,
    help="Success tolerance used with --wait."
)
@click.option(
    "--keep-torque-on-close",
    is_flag=True,
    default=False,
    help="Leave motor torque enabled when this script exits."
)
@click.option(
    "-y", "--yes",
    is_flag=True,
    default=False,
    help="Skip the confirmation prompt."
)
def main(width, unit, velocity, wait, timeout, tolerance_mm, keep_torque_on_close, yes):
    unit = unit.lower()
    target_width_mm = convert_width_to_mm(width, unit)
    validate_width_mm(target_width_mm)

    click.echo(f"UMI gripper calibrated range: 0.0 .. {GRIPPER_MAX_WIDTH:.1f} mm")
    click.echo(f"UMI gripper calibrated range: 0.0 .. {GRIPPER_MAX_WIDTH / 1000.0:.3f} m")
    click.echo(f"Target width: {target_width_mm:.3f} mm ({target_width_mm / 1000.0:.4f} m)")
    click.echo(f"Velocity:     {velocity:.3f}")

    if not yes:
        click.confirm("Send the UMI gripper to this width?", abort=True)

    driver = DYROSBinaryDriver(disable_torque_on_close=not keep_torque_on_close)
    try:
        driver.ack_fault()
        current = driver.script_query()
        click.echo(
            f"Current width: {current['position']:.3f} mm "
            f"({current['position'] / 1000.0:.4f} m)"
        )

        driver.script_position_pd(position=target_width_mm, velocity=velocity)

        if not wait:
            click.echo("Width command sent.")
            return

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state = driver.script_query()
            error_mm = abs(state["position"] - target_width_mm)
            if error_mm <= tolerance_mm:
                click.echo(
                    f"Reached width: {state['position']:.3f} mm "
                    f"({state['position'] / 1000.0:.4f} m)"
                )
                return
            time.sleep(0.05)

        state = driver.script_query()
        raise click.ClickException(
            "Timed out waiting for the UMI gripper to reach the target width. "
            f"Last width: {state['position']:.3f} mm "
            f"({state['position'] / 1000.0:.4f} m)."
        )
    finally:
        driver.close()


if __name__ == "__main__":
    main()
