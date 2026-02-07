# ruff: noqa
"""Test script for DROID RobotEnv control loop.

Runs a fixed spiral trajectory in Cartesian position mode at 15 Hz,
logs commanded vs actual EE poses, then returns to the start position
and cycles the gripper open/close 5 times.

Produces a matplotlib figure saved to --output (default: control_test.png)
with commanded vs actual position, orientation, and timing.

Usage:
    python test_control_loop.py
    python test_control_loop.py --radius 0.03 --revolutions 2 --output my_test.png
"""

import argparse
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from droid.misc.transformations import add_poses
from droid.robot_env import RobotEnv


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(description="DROID control-loop test")
    parser.add_argument(
        "--radius", type=float, default=0.02,
        help="Spiral radius in metres (default 0.02 = 2 cm)",
    )
    parser.add_argument(
        "--revolutions", type=float, default=2.0,
        help="Number of full revolutions for the spiral",
    )
    parser.add_argument(
        "--control_hz", type=float, default=15.0,
        help="Target control rate in Hz",
    )
    parser.add_argument(
        "--duration", type=float, default=4.0,
        help="Duration of the spiral in seconds",
    )
    parser.add_argument(
        "--output", type=str, default="control_test.png",
        help="Path to save the result plot",
    )
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Spiral trajectory generation
# ---------------------------------------------------------------------------

def generate_spiral_deltas(
    radius: float,
    revolutions: float,
    duration: float,
    control_hz: float,
) -> list[np.ndarray]:
    """Generate a sequence of Cartesian deltas tracing a horizontal spiral.

    The spiral is in the XY plane around the starting EE position.
    Z, roll, pitch, yaw deltas are zero.  Gripper stays closed (DROID=1.0).

    Returns a list of 7-D arrays: [dx, dy, dz, droll, dpitch, dyaw, gripper].
    Each delta moves from the *previous target* to the *next target*.
    """
    n_steps = int(duration * control_hz)
    dt = 1.0 / control_hz
    angles = np.linspace(0, 2 * np.pi * revolutions, n_steps + 1)

    # Absolute offsets from the starting position
    xs = radius * np.cos(angles) - radius  # start at offset (0,0)
    ys = radius * np.sin(angles)

    deltas = []
    for i in range(n_steps):
        dx = xs[i + 1] - xs[i]
        dy = ys[i + 1] - ys[i]
        # 7-D: [dx, dy, dz, droll, dpitch, dyaw, gripper_position]
        deltas.append(np.array([dx, dy, 0.0, 0.0, 0.0, 0.0, 1.0]))

    return deltas


# ---------------------------------------------------------------------------
# Main test
# ---------------------------------------------------------------------------

def main():
    args = parse_args()
    dt = 1.0 / args.control_hz

    # --- Initialise robot --------------------------------------------------
    print("Initialising RobotEnv (cartesian_position, gripper=position) ...")
    env = RobotEnv(
        action_space="cartesian_position",
        gripper_action_space="position",
    )
    print("Robot ready.\n")

    # Record starting state
    state_dict, _ = env.get_state()
    start_cartesian = np.array(state_dict["cartesian_position"])
    print(f"Start EE pose: {np.round(start_cartesian, 4).tolist()}")

    # --- Generate spiral ---------------------------------------------------
    deltas = generate_spiral_deltas(
        args.radius, args.revolutions, args.duration, args.control_hz,
    )
    n_steps = len(deltas)
    print(f"Spiral: {n_steps} steps, {args.revolutions} rev, "
          f"r={args.radius} m, {args.duration} s\n")

    # --- Execute spiral & log ----------------------------------------------
    commanded_poses = []   # target cartesian sent to env.step()
    actual_poses = []      # EE pose read back after each step
    timestamps = []        # wall-clock time of each step
    loop_times = []        # actual loop iteration duration

    current_target = start_cartesian.copy()

    print("Running spiral ... (Ctrl-C to abort)")
    t_start = time.monotonic()
    try:
        for i, delta in enumerate(deltas):
            t0 = time.monotonic()

            # Compute next target pose using proper pose composition
            pose_delta = delta[:6]
            gripper = delta[6]
            next_target = add_poses(pose_delta, current_target)

            action = np.concatenate([next_target, [gripper]])
            env.step(action)

            # Read back actual state
            state_dict, _ = env.get_state()
            actual_cart = np.array(state_dict["cartesian_position"])

            commanded_poses.append(next_target.copy())
            actual_poses.append(actual_cart.copy())
            timestamps.append(time.monotonic() - t_start)

            current_target = next_target

            # Rate-limit
            elapsed = time.monotonic() - t0
            loop_times.append(elapsed)
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

            if (i + 1) % 15 == 0:
                print(f"  step {i+1}/{n_steps}  "
                      f"loop={elapsed*1000:.1f} ms  "
                      f"pos_err={np.linalg.norm(actual_cart[:3] - next_target[:3])*1000:.2f} mm")

    except KeyboardInterrupt:
        print("\nAborted by user.")

    print(f"\nSpiral done. Executed {len(commanded_poses)} / {n_steps} steps.")

    # --- Return to start ---------------------------------------------------
    print("Returning to start position (blocking) ...")
    return_action = np.concatenate([start_cartesian, [1.0]])  # gripper closed
    env.update_robot(return_action, action_space="cartesian_position",
                     gripper_action_space="position", blocking=True)
    time.sleep(0.5)

    # Verify we're back
    state_dict, _ = env.get_state()
    return_pose = np.array(state_dict["cartesian_position"])
    return_err = np.linalg.norm(return_pose[:3] - start_cartesian[:3]) * 1000
    print(f"Return position error: {return_err:.2f} mm\n")

    # --- Gripper cycling ---------------------------------------------------
    print("Cycling gripper open/close 5 times ...")
    for cycle in range(5):
        # Open (DROID: 0 = open)
        env._robot.update_gripper(0.0, velocity=False, blocking=True)
        time.sleep(0.3)
        # Close (DROID: 1 = closed)
        env._robot.update_gripper(1.0, velocity=False, blocking=True)
        time.sleep(0.3)
        print(f"  cycle {cycle + 1}/5 done")
    print("Gripper test done.\n")

    # --- Plot results ------------------------------------------------------
    commanded_poses = np.array(commanded_poses)
    actual_poses = np.array(actual_poses)
    timestamps = np.array(timestamps)
    loop_times = np.array(loop_times)

    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle(
        f"Control Loop Test  |  r={args.radius}m  rev={args.revolutions}  "
        f"hz={args.control_hz}  steps={len(commanded_poses)}",
        fontsize=13,
    )

    # --- Top-left: XY trajectory (bird's-eye) ---
    ax = axes[0, 0]
    ax.plot(commanded_poses[:, 0], commanded_poses[:, 1], "b-", label="Commanded", linewidth=1.5)
    ax.plot(actual_poses[:, 0], actual_poses[:, 1], "r--", label="Actual", linewidth=1.0)
    ax.plot(commanded_poses[0, 0], commanded_poses[0, 1], "go", markersize=8, label="Start")
    ax.plot(commanded_poses[-1, 0], commanded_poses[-1, 1], "ms", markersize=8, label="End")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("XY Trajectory")
    ax.legend(fontsize=8)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    # --- Top-right: Position components over time ---
    ax = axes[0, 1]
    labels = ["x", "y", "z"]
    for dim, lbl in enumerate(labels):
        ax.plot(timestamps, commanded_poses[:, dim], "-", label=f"cmd_{lbl}", linewidth=1.2)
        ax.plot(timestamps, actual_poses[:, dim], "--", label=f"act_{lbl}", linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position (m)")
    ax.set_title("Position vs Time")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    # --- Mid-left: Orientation components over time ---
    ax = axes[1, 0]
    labels = ["roll", "pitch", "yaw"]
    for dim, lbl in enumerate(labels):
        ax.plot(timestamps, commanded_poses[:, 3 + dim], "-", label=f"cmd_{lbl}", linewidth=1.2)
        ax.plot(timestamps, actual_poses[:, 3 + dim], "--", label=f"act_{lbl}", linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (rad)")
    ax.set_title("Orientation vs Time")
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)

    # --- Mid-right: Position tracking error ---
    ax = axes[1, 1]
    pos_err = np.linalg.norm(actual_poses[:, :3] - commanded_poses[:, :3], axis=1) * 1000
    ax.plot(timestamps, pos_err, "r-", linewidth=1.0)
    ax.axhline(np.mean(pos_err), color="gray", linestyle="--", label=f"mean={np.mean(pos_err):.2f} mm")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position Error (mm)")
    ax.set_title("Tracking Error")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- Bottom-left: Loop timing ---
    ax = axes[2, 0]
    ax.plot(timestamps, loop_times * 1000, "g-", linewidth=0.8)
    ax.axhline(dt * 1000, color="blue", linestyle="--", label=f"target={dt*1000:.1f} ms")
    ax.axhline(np.mean(loop_times) * 1000, color="red", linestyle="--",
               label=f"mean={np.mean(loop_times)*1000:.1f} ms")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Loop Time (ms)")
    ax.set_title("Loop Timing (excluding sleep)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # --- Bottom-right: Summary stats ---
    ax = axes[2, 1]
    ax.axis("off")
    stats_text = (
        f"Spiral Parameters:\n"
        f"  Radius:       {args.radius*1000:.1f} mm\n"
        f"  Revolutions:  {args.revolutions}\n"
        f"  Duration:     {args.duration:.1f} s\n"
        f"  Control Hz:   {args.control_hz}\n"
        f"  Steps:        {len(commanded_poses)}\n\n"
        f"Tracking Performance:\n"
        f"  Mean pos error:  {np.mean(pos_err):.2f} mm\n"
        f"  Max pos error:   {np.max(pos_err):.2f} mm\n"
        f"  Std pos error:   {np.std(pos_err):.2f} mm\n\n"
        f"Timing:\n"
        f"  Mean loop time:  {np.mean(loop_times)*1000:.1f} ms\n"
        f"  Max loop time:   {np.max(loop_times)*1000:.1f} ms\n"
        f"  Target period:   {dt*1000:.1f} ms\n\n"
        f"Return to start error: {return_err:.2f} mm"
    )
    ax.text(0.05, 0.95, stats_text, transform=ax.transAxes,
            fontsize=10, verticalalignment="top", fontfamily="monospace",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))

    plt.tight_layout()
    plt.savefig(args.output, dpi=150)
    print(f"Plot saved to: {args.output}")


if __name__ == "__main__":
    main()
