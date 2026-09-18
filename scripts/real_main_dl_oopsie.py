"""Collect pi0.5-DROID episodes and annotate each one in the Oopsie browser UI.

Run with --check-only to validate local setup without network or robot connections.
Normal execution initializes/resets the robot, then waits for a browser task.
"""

from __future__ import annotations

import contextlib
import csv
import dataclasses
import datetime
import faulthandler
import logging
import math
import signal
import socket
import time
from pathlib import Path
from typing import Optional
from urllib.error import URLError

import numpy as np
import tyro
from oopsie_data_tools.annotation_tool.rollout_annotator import WebRolloutAnnotator
from oopsie_data_tools.utils import contributor_config
from oopsie_data_tools.utils.robot_profile.robot_profile import load_robot_profile
from oopsie_data_tools.utils.validation.episode_validator import (
    MAX_EPISODE_DURATION_S,
    MIN_EPISODE_DURATION_S,
    MIN_IMAGE_SIZE,
)
from openpi_client import image_tools

SCRIPT_DIR = Path(__file__).resolve().parent


@dataclasses.dataclass
class Args:
    operator_name: str = "Daniel Lawson"

    # Defaults to the operator when the same person performs the annotation.
    annotator_name: Optional[str] = None

    robot_profile: Path = SCRIPT_DIR / "robot_profiles" / "droid_pi05.yaml"

    data_root_dir: Path = Path.home() / "oopsie" / "recordings"

    resume_session_name: Optional[str] = None

    left_camera_id: str = "29712701"

    right_camera_id: str = "23960472"

    wrist_camera_id: str = "11744905"

    external_camera: str = "left"

    remote_host: str = "wss://green.iro.umontreal.ca"

    remote_port: int = 443

    open_loop_horizon: int = 8

    max_timesteps: int = 600

    annotator_port: int = 5003

    open_browser: bool = True

    # No robot/server connection, camera capture, or recording on this path.
    check_only: bool = False


def check_setup(args: Args):
    """Reject incompatible profiles and options before initializing hardware."""
    profile = load_robot_profile(args.robot_profile)
    if not args.operator_name.strip() or args.operator_name.startswith("<"):
        raise ValueError("Provide the human operator's name.")
    if args.annotator_name is not None and not args.annotator_name.strip():
        raise ValueError("Annotator name must be non-empty, or omitted to use the operator.")
    if args.external_camera not in {"left", "right"}:
        raise ValueError("external_camera must be left or right.")
    if args.open_loop_horizon < 1 or args.max_timesteps < 1:
        raise ValueError("open_loop_horizon and max_timesteps must be positive.")
    if not math.isfinite(float(profile.control_freq)) or profile.control_freq <= 0:
        raise ValueError("Profile control_freq must be a positive finite rate.")
    if (
        not MIN_EPISODE_DURATION_S
        <= args.max_timesteps / profile.control_freq
        <= MAX_EPISODE_DURATION_S
    ):
        raise ValueError(
            "max_timesteps must represent between {} and {} seconds at the profile rate.".format(
                MIN_EPISODE_DURATION_S, MAX_EPISODE_DURATION_S
            )
        )
    if profile.is_biarm or profile.uses_mobile_base:
        raise ValueError("This collector supports one stationary Franka arm.")
    if set(profile.action_space) != {"joint_velocity", "gripper_binary"}:
        raise ValueError("This collector records joint_velocity and gripper_binary actions.")
    if len(profile.robot_state_joint_names) != 7 or len(profile.action_joint_names or []) != 7:
        raise ValueError("The profile must identify seven state and action joints.")
    if not set(profile.camera_names).issubset({"left", "right", "wrist"}):
        raise ValueError("Supported recorded cameras are left, right, and wrist.")
    if not set(profile.robot_state_keys).issubset(
        {"joint_position", "cartesian_position", "gripper_position"}
    ):
        raise ValueError("The profile requests state that this collector does not extract.")
    if "cartesian_position" in profile.robot_state_keys:
        if profile.robot_state_orientation_representation != "euler_xyz":
            raise ValueError("DROID provides Cartesian orientation as euler_xyz.")
    if profile.additional_data:
        raise ValueError("This collector does not acquire additional/tactile sensors yet.")
    for name in ("policy_name", "robot_name", "gripper_name"):
        if not str(getattr(profile, name) or "").strip():
            raise ValueError("Profile {} must be filled in.".format(name))
    ids = [args.left_camera_id, args.right_camera_id, args.wrist_camera_id]
    if any(not value.strip() or value.startswith("<") for value in ids) or len(set(ids)) != 3:
        raise ValueError("Provide three distinct camera IDs.")
    if args.resume_session_name and (
        Path(args.resume_session_name).name != args.resume_session_name
        or args.resume_session_name in {".", ".."}
    ):
        raise ValueError("resume_session_name must be a directory name, not a path.")
    lab_id, _ = contributor_config.read_contributor_config()
    print(
        "Profile: {} / {} / {}".format(
            profile.policy_name, profile.robot_name, profile.gripper_name
        )
    )
    print("Lab: {}; operator: {}".format(lab_id, args.operator_name))
    print(
        "Target: {} Hz; replan every {} actions".format(
            profile.control_freq, args.open_loop_horizon
        )
    )
    print(
        "Recorded cameras: {}; policy: {} + wrist".format(
            ", ".join(profile.camera_names), args.external_camera
        )
    )
    print("Output root: {}".format(args.data_root_dir.expanduser().resolve()))
    return profile


@contextlib.contextmanager
def defer_keyboard_interrupt():
    """Finish an inference request or a send-and-record pair before handling Ctrl+C."""
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


def extract_observation(args: Args, raw, camera_names):
    images = {}
    for name in sorted(set(camera_names) | {args.external_camera, "wrist"}):
        camera_id = getattr(args, name + "_camera_id")
        matches = [
            value for key, value in raw["image"].items() if camera_id in key and "left" in key
        ]
        if len(matches) != 1:
            raise ValueError(
                "Expected one left-stereo image for {} ({}), found {}".format(
                    name, camera_id, len(matches)
                )
            )
        frame = np.asarray(matches[0])
        if frame.ndim != 3 or frame.shape[2] not in (3, 4) or frame.dtype != np.uint8:
            raise ValueError(
                "Invalid camera frame for {}: {} {}".format(name, frame.shape, frame.dtype)
            )
        if name in camera_names and min(frame.shape[:2]) < MIN_IMAGE_SIZE:
            raise ValueError(
                "Recorded camera {} must be at least {} pixels in each dimension.".format(
                    name, MIN_IMAGE_SIZE
                )
            )
        # Match real_main_dl.py: discard alpha and convert BGR to RGB. Copy the
        # frame so camera-buffer reuse cannot change an already recorded step.
        images[name] = frame[..., :3][..., ::-1].copy()
    state = raw["robot_state"]
    values = {
        "joint_position": np.asarray(state["joint_positions"], dtype=np.float64),
        "cartesian_position": np.asarray(state["cartesian_position"], dtype=np.float64),
        "gripper_position": np.asarray([state["gripper_position"]], dtype=np.float64),
    }
    for name, shape in (
        ("joint_position", (7,)),
        ("cartesian_position", (6,)),
        ("gripper_position", (1,)),
    ):
        if values[name].shape != shape or not np.isfinite(values[name]).all():
            raise ValueError("Invalid {}: expected finite values with shape {}".format(name, shape))
    return {"image_observation": images, "robot_state": values}


def policy_request(args: Args, observation, instruction):
    images, state = observation["image_observation"], observation["robot_state"]
    return {
        "observation/exterior_image_1_left": image_tools.resize_with_pad(
            images[args.external_camera], 224, 224
        ),
        "observation/wrist_image_left": image_tools.resize_with_pad(images["wrist"], 224, 224),
        "observation/joint_position": state["joint_position"],
        "observation/gripper_position": state["gripper_position"],
        "prompt": instruction,
    }


def validate_chunk(actions, horizon):
    chunk = np.asarray(actions, dtype=np.float64)
    if chunk.ndim != 2 or chunk.shape[1] != 8 or chunk.shape[0] < horizon:
        raise ValueError(
            "Expected action chunk (T, 8), T >= {}; got {}".format(horizon, chunk.shape)
        )
    if not np.isfinite(chunk).all():
        raise ValueError("Policy returned NaN/Inf actions.")
    return chunk


def run_episode(args, profile, env, policy, recorder, instruction):
    """Record each successfully dispatched action with its pre-action observation."""
    if not instruction.strip():
        raise ValueError("Task instruction must be non-empty.")
    command_times = []
    inference_times = []
    status, error = "completed", ""
    chunk, chunk_index = None, 0
    started = time.perf_counter()
    period = 1.0 / profile.control_freq
    try:
        for _ in range(args.max_timesteps):
            observation = extract_observation(args, env.get_observation(), profile.camera_names)
            if chunk is None or chunk_index >= args.open_loop_horizon:
                request = policy_request(args, observation, instruction)
                with defer_keyboard_interrupt():
                    inference_started = time.perf_counter()
                    response = policy.infer(request)
                    inference_times.append(time.perf_counter() - inference_started)
                chunk = validate_chunk(response["actions"], args.open_loop_horizon)
                chunk_index = 0
            action = np.clip(chunk[chunk_index].copy(), -1.0, 1.0)
            action[-1] = float(chunk[chunk_index, -1] > 0.5)
            chunk_index += 1
            # Pace command dispatches, including chunk boundaries. If inference
            # overruns, resume without issuing a burst of catch-up commands.
            if command_times:
                remaining = period - (time.perf_counter() - command_times[-1])
                if remaining > 0:
                    time.sleep(remaining)
            with defer_keyboard_interrupt():
                dispatched = time.perf_counter()
                env.step(action)
                recorder.record_step(
                    observation=observation,
                    action={
                        "joint_velocity": action[:7].copy(),
                        "gripper_binary": action[7:].copy(),
                    },
                )
                command_times.append(dispatched)
    except KeyboardInterrupt:
        status = "interrupted"
    except Exception as exc:
        status, error = "error", "{}: {}".format(type(exc).__name__, exc)
        logging.exception("Rollout stopped; preserving the previously recorded steps.")
    intervals = np.diff(command_times)
    summary = {
        "steps": len(command_times),
        "status": status,
        "error": error,
        "duration_seconds": time.perf_counter() - started,
        "target_hz": profile.control_freq,
        "measured_hz": float(1.0 / intervals.mean()) if len(intervals) else None,
        "interval_p95_ms": float(np.percentile(intervals, 95) * 1000) if len(intervals) else None,
        "mean_inference_ms": float(np.mean(inference_times) * 1000) if inference_times else None,
    }
    print("Rollout: {steps} steps; {status}; target {target_hz} Hz".format(**summary))
    if summary["measured_hz"] is not None:
        print(
            "Measured command rate: {:.2f} Hz; p95 interval: {:.1f} ms".format(
                summary["measured_hz"], summary["interval_p95_ms"]
            )
        )
    if error:
        print(error)
    return summary


def append_summary(path, episode_id, summary):
    new_file = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as stream:
        row = {"episode_id": episode_id, **summary}
        writer = csv.DictWriter(stream, fieldnames=list(row))
        if new_file:
            writer.writeheader()
        writer.writerow(row)


def choose_keep_episode():
    """Require an explicit decision before writing episode data."""
    while True:
        choice = input("Keep this episode or discard it? [k/d]: ").strip().lower()
        if choice in {"k", "keep"}:
            return True
        if choice in {"d", "discard"}:
            return False
        print("Enter k to save and annotate, or d to discard without saving.")


def main(args: Args):
    profile = check_setup(args)
    # Imports verify the installed stack but do not construct devices or clients.
    from openpi_client.websocket_client_policy import WebsocketClientPolicy

    from droid.robot_env import RobotEnv

    if args.check_only:
        print("Local checks passed. No robot, cameras, or servers were contacted.")
        return
    session_name = args.resume_session_name or datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    root = args.data_root_dir.expanduser().resolve()
    session_dir = root / session_name
    # The upstream helper reuses any existing server on this port, which could
    # belong to a different annotator or data root. Require our own fresh server.
    with socket.socket() as probe:
        if probe.connect_ex(("localhost", args.annotator_port)) == 0:
            raise RuntimeError(
                "Annotation port {} is occupied. Stop the previous annotation server "
                "or select another --annotator-port.".format(args.annotator_port)
            )
    annotator = WebRolloutAnnotator(
        robot_profile=profile,
        data_root_dir=root,
        operator_name=args.operator_name.strip(),
        annotator_name=(args.annotator_name or args.operator_name).strip(),
        resume_session_name=session_name,
        port=args.annotator_port,
        wait_for_annotation=True,
        open_browser=args.open_browser,
    )
    try:
        annotator.start()
        print("Annotation UI: http://localhost:{}".format(args.annotator_port))
        print("Session: {}".format(session_dir))
        policy = WebsocketClientPolicy(args.remote_host, args.remote_port)
        print("Initializing DROID (this resets the robot to its configured start pose).")
        env = RobotEnv(action_space="joint_velocity", gripper_action_space="position")
        while True:
            print("Submit the next instruction in the browser. Ctrl+C ends an active rollout.")
            instruction = annotator.wait_for_task()
            annotator.reset_episode_recorder()
            episode_id = annotator.episode_name
            summary = run_episode(args, profile, env, policy, annotator, instruction)
            append_summary(session_dir / "rollouts.csv", episode_id, summary)
            if summary["steps"] / profile.control_freq < MIN_EPISODE_DURATION_S:
                print(
                    "Too few steps for a valid episode (minimum {}). No episode saved; "
                    "rollout diagnostics are in rollouts.csv.".format(
                        math.ceil(MIN_EPISODE_DURATION_S * profile.control_freq)
                    )
                )
                break
            if choose_keep_episode():
                print("Saving episode, then waiting for your annotation in the browser.")
                annotator.finish_rollout(instruction=instruction)
                print("Saved and annotated: {}".format(session_dir / (episode_id + ".h5")))
            else:
                annotator.reset_episode_recorder()
                # The installed helper has no public discard method. End the
                # browser task without invoking its save-and-annotate workflow.
                print(
                    "Discarded: no HDF5, videos, or annotation saved. "
                    "Only the diagnostic entry in rollouts.csv remains."
                )
                try:
                    annotator._api_post("/api/task/done", {})
                except (URLError, OSError) as exc:
                    print(
                        "Annotation server unavailable ({}). Discard succeeded; "
                        "ending the session. Restart the collector to continue.".format(exc)
                    )
                    break
            if summary["status"] == "error":
                print("Ending the session after the rollout error; inspect it before restarting.")
                break
            if input("Reset robot and collect another episode? [y/N]: ").strip().lower() != "y":
                break
            env.reset()
    except KeyboardInterrupt:
        print(
            "Session ended. Unsaved rollout data is discarded; already saved episodes remain on disk."
        )
    finally:
        annotator.stop()
        print("Recordings: {}".format(session_dir))


if __name__ == "__main__":
    faulthandler.enable()
    logging.basicConfig(level=logging.INFO)
    main(tyro.cli(Args))
