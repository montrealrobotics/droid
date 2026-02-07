# ruff: noqa
"""MolmoAct Franka Robot Client.

Connects to a MolmoAct WebSocket server, collects observations from the
Franka arm via DROID's RobotEnv, queries the policy, and applies the
returned Cartesian delta actions.

Usage:
    python molmoact_client.py \
        --server_host 172.19.0.76 \
        --server_port 8000

Requires:
    - droid (DROID robot stack)
    - websockets >= 11.0 (for websockets.sync)
    - numpy, Pillow
"""

import argparse
import base64
import contextlib
import io
import json
import signal
import time

import numpy as np
from PIL import Image

from droid.misc.transformations import add_poses
from droid.robot_env import RobotEnv


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(description="MolmoAct Franka Robot Client")

    # Camera IDs (match your physical setup)
    parser.add_argument("--right_camera_id", type=str, default="24078426")
    parser.add_argument("--left_camera_id", type=str, default="23122133")
    parser.add_argument("--wrist_camera_id", type=str, default="14301394")

    # Server
    parser.add_argument(
        "--server_host",
        type=str,
        default="172.19.0.76",
        help="IP of the MolmoAct server (omegaduck)",
    )
    parser.add_argument("--server_port", type=int, default=8000)

    # Rollout
    parser.add_argument(
        "--max_timesteps",
        type=int,
        default=600,
        help="Maximum number of action steps per rollout",
    )
    parser.add_argument(
        "--control_hz",
        type=float,
        default=15.0,
        help="Control frequency (Hz).  A sleep is inserted between actions "
             "so that the effective rate matches the DROID training data.",
    )

    return parser.parse_args()


# ---------------------------------------------------------------------------
# Keyboard-interrupt guard (same pattern as pi_main2.py)
# ---------------------------------------------------------------------------

@contextlib.contextmanager
def prevent_keyboard_interrupt():
    """Delay Ctrl-C until after the protected code block completes."""
    interrupted = False
    original_handler = signal.getsignal(signal.SIGINT)

    def _handler(signum, frame):
        nonlocal interrupted
        interrupted = True

    signal.signal(signal.SIGINT, _handler)
    try:
        yield
    finally:
        signal.signal(signal.SIGINT, original_handler)
        if interrupted:
            raise KeyboardInterrupt


# ---------------------------------------------------------------------------
# Observation helpers
# ---------------------------------------------------------------------------

def extract_observation(args, obs_dict: dict) -> dict:
    """Pull images and robot state out of the raw DROID observation.

    Images are returned as uint8 RGB numpy arrays.
    """
    image_observations = obs_dict["image"]
    left_image = right_image = wrist_image = None

    for key in image_observations:
        # "left" here refers to the left lens of the stereo pair
        if args.left_camera_id in key and "left" in key:
            left_image = image_observations[key]
        elif args.right_camera_id in key and "left" in key:
            right_image = image_observations[key]
        elif args.wrist_camera_id in key and "left" in key:
            wrist_image = image_observations[key]

    # Drop alpha channel, convert BGR → RGB
    left_image = left_image[..., :3][..., ::-1]
    right_image = right_image[..., :3][..., ::-1]
    wrist_image = wrist_image[..., :3][..., ::-1]

    robot_state = obs_dict["robot_state"]

    return {
        "left_image": np.ascontiguousarray(left_image),
        "right_image": np.ascontiguousarray(right_image),
        "wrist_image": np.ascontiguousarray(wrist_image),
        "cartesian_position": np.array(robot_state["cartesian_position"]),
        "gripper_position": robot_state["gripper_position"],
    }


def numpy_to_base64_png(img_array: np.ndarray) -> str:
    """Encode an RGB uint8 numpy image to a base64 PNG string."""
    buf = io.BytesIO()
    Image.fromarray(img_array).save(buf, format="PNG")
    return base64.b64encode(buf.getvalue()).decode("ascii")


# ---------------------------------------------------------------------------
# Action conversion
# ---------------------------------------------------------------------------

def convert_action_for_droid(
    current_cartesian: np.ndarray,
    molmoact_action: list,
) -> np.ndarray:
    """Convert a single MolmoAct Cartesian delta into a DROID-compatible action.

    MolmoAct outputs:
        [dx, dy, dz, droll, dpitch, dyaw, gripper_normalized]
        - Dims 0-5: Cartesian deltas (metres / radians)
        - Dim 6:    Normalized gripper value in [-1, 1] where
                    -1 = closed, +1 = open  (mask=False → no unnorm)

    DROID cartesian_position action (DoF=7):
        [target_x, target_y, target_z, target_roll, target_pitch, target_yaw,
         gripper_position]
        - Gripper position in [0, 1] where 1 = closed, 0 = open.

    Returns:
        np.ndarray of shape (7,) ready for env.step().
    """
    delta_pose = np.array(molmoact_action[:6], dtype=np.float64)

    # Compose poses properly (uses scipy Rotation for orientation)
    target_pose = add_poses(delta_pose, current_cartesian)

    # Gripper conversion:
    #   MolmoAct normalised: -1 = closed, +1 = open
    #   DROID position:       1 = closed,  0 = open
    # Binarise at 0 then invert.
    gripper_norm = molmoact_action[6]
    if gripper_norm > 0:
        droid_gripper = 0.0  # open
    else:
        droid_gripper = 1.0  # closed

    return np.concatenate([target_pose, [droid_gripper]])


# ---------------------------------------------------------------------------
# Server communication
# ---------------------------------------------------------------------------

def query_server(
    ws,
    left_image: np.ndarray,
    right_image: np.ndarray,
    wrist_image: np.ndarray,
    instruction: str,
) -> dict:
    """Send all camera images to the MolmoAct server and return the response.

    The server decides which images to use based on its --camera_config.
    Returns dict with keys: actions, trace, depth, raw_text.
    Raises RuntimeError on server-side errors.
    """
    request = json.dumps({
        "left_image": numpy_to_base64_png(left_image),
        "right_image": numpy_to_base64_png(right_image),
        "wrist_image": numpy_to_base64_png(wrist_image),
        "instruction": instruction,
    })
    ws.send(request)
    response = json.loads(ws.recv())

    if "error" in response:
        raise RuntimeError(f"Server error: {response['error']}")

    return response


# ---------------------------------------------------------------------------
# Main rollout loop
# ---------------------------------------------------------------------------

def run_rollout(args):
    from websockets.sync.client import connect

    # Initialise robot with Cartesian position control + absolute gripper
    env = RobotEnv(
        action_space="cartesian_position",
        gripper_action_space="position",
    )
    print("Robot environment initialised.")

    uri = f"ws://{args.server_host}:{args.server_port}"
    print(f"Connecting to MolmoAct server at {uri} ...")

    with connect(uri, max_size=50 * 1024 * 1024) as ws:
        print("Connected.\n")

        while True:
            instruction = input("Enter instruction (or 'quit'): ").strip()
            if instruction.lower() in ("quit", "q", "exit"):
                break

            step_count = 0
            dt = 1.0 / args.control_hz
            print(f"Running rollout: '{instruction}'  (Ctrl-C to stop early)")

            try:
                while step_count < args.max_timesteps:
                    # --- Observe -------------------------------------------
                    obs = extract_observation(args, env.get_observation())

                    # --- Query policy server (protected from Ctrl-C) -------
                    with prevent_keyboard_interrupt():
                        response = query_server(
                            ws,
                            obs["left_image"],
                            obs["right_image"],
                            obs["wrist_image"],
                            instruction,
                        )

                    actions = response["actions"]
                    if not actions:
                        print("WARNING: server returned 0 actions, re-querying ...")
                        continue

                    print(
                        f"  [step {step_count}] received {len(actions)} action chunk(s), executing first"
                    )

                    # --- Execute only the first action chunk ---------------
                    action = actions[0]

                    # Read fresh EE pose
                    state_dict, _ = env.get_state()
                    current_cartesian = np.array(
                        state_dict["cartesian_position"]
                    )

                    droid_action = convert_action_for_droid(
                        current_cartesian, action,
                    )

                    t0 = time.monotonic()
                    env.step(droid_action)
                    step_count += 1

                    # Rate-limit to match training control frequency
                    elapsed = time.monotonic() - t0
                    sleep_time = dt - elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)

            except KeyboardInterrupt:
                print("\nRollout stopped by user.")

            print(f"Rollout finished after {step_count} steps.\n")

    print("Disconnected from server.")


# ---------------------------------------------------------------------------

def main():
    args = parse_args()
    run_rollout(args)


if __name__ == "__main__":
    main()
