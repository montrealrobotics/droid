"""Offline collector checks: no robot construction or network requests."""

import dataclasses
import importlib.util
import sys
from pathlib import Path

import h5py
import numpy as np
import pytest
from oopsie_data_tools.annotation_tool.episode_recorder import EpisodeRecorder
from oopsie_data_tools.utils import contributor_config
from oopsie_data_tools.utils.robot_profile.robot_profile import load_robot_profile
from oopsie_data_tools.utils.validation.validation_utils import validate_h5_file

SCRIPT = Path(__file__).resolve().parents[1] / "real_main_dl_oopsie.py"
spec = importlib.util.spec_from_file_location("collector_under_test", str(SCRIPT))
collector = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = collector
spec.loader.exec_module(collector)


@pytest.fixture
def profile():
    return load_robot_profile(collector.Args().robot_profile)


@pytest.fixture
def clock(monkeypatch):
    class Clock:
        now = 0.0

        def perf_counter(self):
            return self.now

        def sleep(self, duration):
            assert duration >= 0
            self.now += duration

    fake = Clock()
    monkeypatch.setattr(collector, "time", fake)
    return fake


def raw_observation():
    args = collector.Args()
    images = {}
    for index, name in enumerate(("left", "right", "wrist")):
        frame = np.full((224, 224, 4), 20 + index, dtype=np.uint8)
        frame[..., 0] = 10 + index
        frame[..., 2] = 30 + index
        images[getattr(args, name + "_camera_id") + "_left"] = frame
    return {
        "image": images,
        "robot_state": {
            "joint_positions": np.zeros(7),
            "cartesian_position": np.zeros(6),
            "gripper_position": 0.25,
        },
    }


class FakeEnv:
    def __init__(self, clock, fail_at=None, interrupt_at=None):
        self.clock = clock
        self.actions = []
        self.fail_at = fail_at
        self.interrupt_at = interrupt_at

    def get_observation(self):
        if len(self.actions) == self.interrupt_at:
            raise KeyboardInterrupt
        self.clock.now += 0.01
        return raw_observation()

    def step(self, action):
        if len(self.actions) == self.fail_at:
            raise RuntimeError("simulated command failure")
        self.actions.append(action.copy())
        self.clock.now += 0.001


class FakePolicy:
    def __init__(self, clock, actions=None):
        self.clock = clock
        self.requests = []
        self.actions = np.zeros((15, 8)) if actions is None else actions

    def infer(self, request):
        self.requests.append(request)
        self.clock.now += 0.2
        return {"actions": self.actions}


class FakeRecorder:
    def __init__(self):
        self.steps = []

    def record_step(self, observation, action):
        self.steps.append((observation, action))


@pytest.mark.parametrize("shape", [(15, 8), (10, 8), (8, 8)])
def test_accepts_server_chunk_lengths(shape):
    assert collector.validate_chunk(np.zeros(shape), 8).shape == shape


@pytest.mark.parametrize(
    "actions",
    [
        np.zeros((7, 8)),
        np.zeros((15, 7)),
        np.zeros(8),
        np.full((15, 8), np.nan),
        np.full((15, 8), np.inf),
    ],
)
def test_invalid_chunk_never_reaches_robot(actions, profile, clock):
    env, recorder = FakeEnv(clock), FakeRecorder()
    summary = collector.run_episode(
        collector.Args(max_timesteps=1),
        profile,
        env,
        FakePolicy(clock, actions),
        recorder,
        "test task",
    )
    assert summary["status"] == "error"
    assert summary["steps"] == 0
    assert env.actions == [] and recorder.steps == []


def test_camera_conversion_copies_frames_and_policy_uses_only_selected_views():
    raw = raw_observation()
    args = collector.Args(external_camera="right")
    obs = collector.extract_observation(args, raw, ["left", "right", "wrist"])
    assert obs["image_observation"]["right"][0, 0].tolist() == [31, 21, 11]
    request = collector.policy_request(args, obs, "test task")
    assert set(request) == {
        "observation/exterior_image_1_left",
        "observation/wrist_image_left",
        "observation/joint_position",
        "observation/gripper_position",
        "prompt",
    }
    assert request["observation/exterior_image_1_left"][0, 0].tolist() == [31, 21, 11]
    for image in raw["image"].values():
        image[:] = 0
    assert obs["image_observation"]["right"][0, 0].tolist() == [31, 21, 11]


def test_missing_camera_fails_before_action(profile, clock):
    env = FakeEnv(clock)
    raw = raw_observation()
    raw["image"].pop(collector.Args().wrist_camera_id + "_left")
    env.get_observation = lambda: raw
    summary = collector.run_episode(
        collector.Args(max_timesteps=1),
        profile,
        env,
        FakePolicy(clock),
        FakeRecorder(),
        "test task",
    )
    assert summary["status"] == "error"
    assert env.actions == []


def test_chunks_replan_and_record_exact_clipped_binary_commands(profile, clock):
    actions = np.ones((15, 8)) * 2
    actions[:, 0] = -2
    actions[::2, -1] = 0.5
    env, policy, recorder = FakeEnv(clock), FakePolicy(clock, actions), FakeRecorder()
    summary = collector.run_episode(
        collector.Args(max_timesteps=17), profile, env, policy, recorder, "test task"
    )
    assert len(policy.requests) == 3
    assert summary["steps"] == 17 and summary["status"] == "completed"
    assert summary["measured_hz"] < 15  # synchronous inference creates gaps
    assert all(action[0] == -1 and action[1] == 1 for action in env.actions)
    assert [action[-1] for action in env.actions[:4]] == [0, 1, 0, 1]
    for sent, (_, recorded) in zip(env.actions, recorder.steps):
        np.testing.assert_array_equal(
            sent, np.concatenate([recorded["joint_velocity"], recorded["gripper_binary"]])
        )


def test_command_intervals_are_paced_even_across_slow_inference(profile, clock):
    env, policy = FakeEnv(clock), FakePolicy(clock)
    dispatch_times = []
    original_step = env.step

    def step(action):
        dispatch_times.append(clock.now)
        original_step(action)

    env.step = step
    collector.run_episode(
        collector.Args(max_timesteps=10), profile, env, policy, FakeRecorder(), "test task"
    )
    intervals = np.diff(dispatch_times)
    assert np.all(intervals >= 1.0 / 15 - 1e-10)
    assert intervals[7] > 0.2


@pytest.mark.parametrize("mode,status", [("interrupt_at", "interrupted"), ("fail_at", "error")])
def test_interruption_or_failure_retains_completed_steps(mode, status, profile, clock):
    env = FakeEnv(clock, **{mode: 2})
    recorder = FakeRecorder()
    summary = collector.run_episode(
        collector.Args(max_timesteps=5), profile, env, FakePolicy(clock), recorder, "test task"
    )
    assert summary["status"] == status
    assert summary["steps"] == len(recorder.steps) == 2


def test_real_recorder_writes_valid_hdf5_and_three_videos(tmp_path, monkeypatch, profile, clock):
    monkeypatch.setattr(
        contributor_config, "read_contributor_config", lambda: ("test_lab", "unused_test_token")
    )
    recorder = EpisodeRecorder(profile, tmp_path, operator_name="Test Operator")
    args = collector.Args(max_timesteps=15)
    env = FakeEnv(clock)
    summary = collector.run_episode(args, profile, env, FakePolicy(clock), recorder, "test task")
    assert summary["steps"] == 15
    recorder.finish_rollout(instruction="test task", success=1.0)
    h5_path = recorder.session_dir / (recorder.save_fname + ".h5")
    validate_h5_file(str(h5_path), strict_annotation_check=True)
    assert len(list(recorder.session_dir.glob("*.mp4"))) == 3
    with h5py.File(str(h5_path), "r") as f:
        assert f["actions/joint_velocity"].shape == (15, 7)
        assert f["actions/gripper_binary"].shape == (15, 1)
        assert f["observations/robot_states/cartesian_position"].shape == (15, 7)
        assert set(f["observations/video_paths"]) == {"left", "right", "wrist"}
        assert f.attrs["operator_name"] == "Test Operator"


def test_profile_mismatch_rejected_before_robot_initialization(monkeypatch, profile):
    monkeypatch.setattr(
        collector,
        "load_robot_profile",
        lambda _: dataclasses.replace(
            profile, action_space=["cartesian_velocity", "gripper_binary"]
        ),
    )
    with pytest.raises(ValueError, match="joint_velocity"):
        collector.check_setup(collector.Args())


def test_check_only_does_not_construct_hardware_or_connect(monkeypatch):
    from openpi_client import websocket_client_policy

    import droid.robot_env

    def forbidden(*args, **kwargs):
        pytest.fail("check-only must not construct hardware or connect to a server")

    monkeypatch.setattr(droid.robot_env, "RobotEnv", forbidden)
    monkeypatch.setattr(websocket_client_policy, "WebsocketClientPolicy", forbidden)
    monkeypatch.setattr(collector, "WebRolloutAnnotator", forbidden)
    monkeypatch.setattr(
        contributor_config, "read_contributor_config", lambda: ("test_lab", "unused_test_token")
    )
    collector.main(collector.Args(check_only=True))


@pytest.mark.parametrize("steps", [1, 14, 9001])
def test_invalid_episode_length_rejected_before_hardware(steps):
    with pytest.raises(ValueError, match="max_timesteps"):
        collector.check_setup(collector.Args(max_timesteps=steps))


@pytest.mark.parametrize("choice", ["keep", "discard", "discard_offline"])
@pytest.mark.parametrize("fail", [False, True])
def test_main_keep_or_discard_before_saving(tmp_path, monkeypatch, profile, clock, choice, fail):
    from openpi_client import websocket_client_policy

    import droid.robot_env

    monkeypatch.setattr(collector, "check_setup", lambda _: profile)
    monkeypatch.setattr(
        contributor_config, "read_contributor_config", lambda: ("test_lab", "unused_test_token")
    )
    env = FakeEnv(clock, fail_at=15 if fail else None)
    monkeypatch.setattr(droid.robot_env, "RobotEnv", lambda **kwargs: env)
    monkeypatch.setattr(
        websocket_client_policy, "WebsocketClientPolicy", lambda *args: FakePolicy(clock)
    )

    class FreePort:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def connect_ex(self, address):
            return 1

    monkeypatch.setattr(collector.socket, "socket", FreePort)
    events, instances = [], []
    real_annotator = collector.WebRolloutAnnotator

    class LocalAnnotator(real_annotator):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            instances.append(self)

        def start(self):
            pass

        def stop(self):
            events.append("stop")

        def wait_for_task(self):
            return "test task"

        def _api_post(self, path, payload):
            assert path == "/api/task/done" and payload == {}
            events.append("idle")
            if choice == "discard_offline":
                raise collector.URLError(ConnectionRefusedError("simulated server loss"))

        def finish_rollout(self, instruction):
            events.append("save")
            # Simulate the human annotation only in this offline test.
            self._active_recorder.finish_rollout(instruction=instruction, success=1.0)

    monkeypatch.setattr(collector, "WebRolloutAnnotator", LocalAnnotator)
    answer = "discard" if choice == "discard_offline" else choice
    answers = iter(["", "invalid", answer] + ([] if fail or choice == "discard_offline" else ["n"]))
    monkeypatch.setattr("builtins.input", lambda _: next(answers))
    collector.main(collector.Args(data_root_dir=tmp_path, max_timesteps=17 if fail else 15))
    assert len(list(tmp_path.rglob("rollouts.csv"))) == 1
    assert events == (["save", "stop"] if choice == "keep" else ["idle", "stop"])
    assert len(list(tmp_path.rglob("*.h5"))) == (1 if choice == "keep" else 0)
    assert len(list(tmp_path.rglob("*.mp4"))) == (3 if choice == "keep" else 0)
    if choice != "keep":
        recorder = instances[0]._active_recorder
        assert not recorder.timesteps and all(not frames for frames in recorder.frames.values())
        assert not list(tmp_path.rglob("*.json"))


def test_discard_then_keep_next_episode(tmp_path, monkeypatch, profile, clock):
    from openpi_client import websocket_client_policy

    import droid.robot_env

    monkeypatch.setattr(collector, "check_setup", lambda _: profile)
    monkeypatch.setattr(
        contributor_config, "read_contributor_config", lambda: ("test_lab", "unused_test_token")
    )
    events = []
    env = FakeEnv(clock)
    env.reset = lambda: events.append("reset_robot")
    monkeypatch.setattr(droid.robot_env, "RobotEnv", lambda **kwargs: env)
    monkeypatch.setattr(
        websocket_client_policy, "WebsocketClientPolicy", lambda *args: FakePolicy(clock)
    )

    class FreePort:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            pass

        def connect_ex(self, address):
            return 1

    monkeypatch.setattr(collector.socket, "socket", FreePort)

    class LocalAnnotator(collector.WebRolloutAnnotator):
        def start(self):
            pass

        def stop(self):
            pass

        def wait_for_task(self):
            events.append("task")
            return "test task"

        def _api_post(self, path, payload):
            assert path == "/api/task/done"
            events.append("idle")

        def finish_rollout(self, instruction):
            events.append("save")
            assert len(self._active_recorder.timesteps) == 15
            self._active_recorder.finish_rollout(instruction=instruction, success=1.0)

    monkeypatch.setattr(collector, "WebRolloutAnnotator", LocalAnnotator)
    answers = iter(["d", "y", "k", "n"])
    monkeypatch.setattr("builtins.input", lambda _: next(answers))
    collector.main(collector.Args(data_root_dir=tmp_path, max_timesteps=15))
    assert events == ["task", "idle", "reset_robot", "task", "save"]
    assert len(list(tmp_path.rglob("*.h5"))) == 1
    assert len(list(tmp_path.rglob("*.mp4"))) == 3
