"""Interactive pi0.5-DROID demo; no recordings or annotation dependencies.

Ctrl+C ends a rollout and returns to the task prompt. In-flight inference and
robot RPC calls finish first. The robot holds its last commanded target; this
is not an emergency stop. New tasks start from the current pose without reset.
"""

from __future__ import annotations

import contextlib
import dataclasses
import faulthandler
import signal

import numpy as np
import tyro
from openpi_client import image_tools


@dataclasses.dataclass
class Args:
    right_camera_id: str = "23960472"

    left_camera_id: str = "29712701"

    wrist_camera_id: str = "11744905"

    external_camera: str = "left"

    # A rollout ends at this action limit; task success is not detected automatically.
    max_timesteps: int = 600

    open_loop_horizon: int = 8

    remote_host: str = "wss://green.iro.umontreal.ca"

    remote_port: int = 443

    # Match the original runner's initialization reset. Disable to start in place.
    reset_on_start: bool = True

    # Check imports/options only; do not construct robot or policy connections.
    check_only: bool = False


@contextlib.contextmanager
def defer_keyboard_interrupt():
    """Drain an in-flight call before returning to the prompt, avoiding stale replies."""
    interrupted = False
    original = signal.getsignal(signal.SIGINT)

    def handler(signum, frame):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, handler)
    try:
        yield
    finally:
        signal.signal(signal.SIGINT, original)
        if interrupted:
            raise KeyboardInterrupt


def policy_request(args, raw, instruction):
    images = {}
    for name in (args.external_camera, "wrist"):
        camera_id = getattr(args, name + "_camera_id")
        matches = [v for k, v in raw["image"].items() if camera_id in k and "left" in k]
        if len(matches) != 1:
            raise ValueError("Expected one left-stereo frame for {} ({}).".format(name, camera_id))
        frame = np.asarray(matches[0])
        if frame.ndim != 3 or frame.shape[-1] not in (3, 4) or frame.dtype != np.uint8:
            raise ValueError("Invalid camera frame for {}.".format(name))
        images[name] = image_tools.resize_with_pad(frame[..., :3][..., ::-1].copy(), 224, 224)
    state = raw["robot_state"]
    joints = np.asarray(state["joint_positions"], dtype=np.float64)
    gripper = np.asarray([state["gripper_position"]], dtype=np.float64)
    if joints.shape != (7,) or gripper.shape != (1,):
        raise ValueError("Expected seven joint positions and one gripper position.")
    if not np.isfinite(joints).all() or not np.isfinite(gripper).all():
        raise ValueError("Robot state contains NaN/Inf.")
    return {
        "observation/exterior_image_1_left": images[args.external_camera],
        "observation/wrist_image_left": images["wrist"],
        "observation/joint_position": joints,
        "observation/gripper_position": gripper,
        "prompt": instruction,
    }


def run_task(args, env, policy, instruction):
    """Execute fresh chunks for one instruction, matching the original unpaced loop."""
    chunk, index, sent = None, 0, 0
    print(
        "Running: {}\nCtrl+C stops this rollout; limit: {} actions.".format(
            instruction, args.max_timesteps
        )
    )
    try:
        for _ in range(args.max_timesteps):
            raw = env.get_observation()
            if chunk is None or index >= args.open_loop_horizon:
                request = policy_request(args, raw, instruction)
                with defer_keyboard_interrupt():
                    response = policy.infer(request)
                chunk = np.asarray(response["actions"], dtype=np.float64)
                if (
                    chunk.ndim != 2
                    or chunk.shape[1] != 8
                    or chunk.shape[0] < args.open_loop_horizon
                ):
                    raise ValueError(
                        "Expected action chunk (T, 8), T >= {}; got {}.".format(
                            args.open_loop_horizon, chunk.shape
                        )
                    )
                if not np.isfinite(chunk).all():
                    raise ValueError("Policy returned NaN/Inf actions.")
                index = 0
            action = np.clip(chunk[index].copy(), -1.0, 1.0)
            action[-1] = float(chunk[index, -1] > 0.5)
            with defer_keyboard_interrupt():
                env.step(action)
                sent += 1
            index += 1
    except KeyboardInterrupt:
        print("Task interrupted after {} actions. No further actions will be sent.".format(sent))
        return
    print("Action limit reached ({}). Enter the next task when ready.".format(sent))


def task_loop(args, env, policy):
    print("Enter a task; /reset opens the gripper and returns to the start pose; /quit exits.")
    print("New tasks use the current robot pose. No automatic reset between tasks.")
    while True:
        try:
            instruction = input("Task> ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\nDemo finished.")
            return
        if instruction.lower() in {"/quit", "/exit"}:
            print("Demo finished.")
            return
        if instruction.lower() == "/reset":
            print("Resetting: opening the gripper and moving to the configured start pose...")
            env.reset()
            print("Reset complete. Enter the next task.")
            continue
        if not instruction:
            continue
        run_task(args, env, policy, instruction)


def main(args):
    if args.external_camera not in {"left", "right"}:
        raise ValueError("external_camera must be left or right.")
    if args.max_timesteps < 1 or args.open_loop_horizon < 1:
        raise ValueError("max_timesteps and open_loop_horizon must be positive.")
    from openpi_client.websocket_client_policy import WebsocketClientPolicy

    from droid.robot_env import RobotEnv

    if args.check_only:
        print("Local checks passed. No hardware/server connections or files created.")
        return
    try:
        policy = WebsocketClientPolicy(args.remote_host, args.remote_port)
        print(
            "Initializing DROID{}.".format(
                " and resetting to its start pose" if args.reset_on_start else " without resetting"
            )
        )
        env = RobotEnv(
            action_space="joint_velocity",
            gripper_action_space="position",
            do_reset=args.reset_on_start,
        )
        task_loop(args, env, policy)
    except KeyboardInterrupt:
        print("\nDemo stopped.")
    except Exception as exc:
        print("Demo stopped after an error: {}: {}".format(type(exc).__name__, exc))
        print("No further actions will be sent. Inspect the error before restarting.")
        raise


if __name__ == "__main__":
    faulthandler.enable()
    main(tyro.cli(Args))
