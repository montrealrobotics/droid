# Collect pi0.5-DROID episodes

Use the Python 3.7 `robot_oopsie` environment with the installed Oopsie compatibility
port. The collector uses `robot_profiles/droid_pi05.yaml` and your existing
Oopsie contributor configuration.

Check local dependencies and configuration without connecting to hardware:

```bash
conda activate robot_oopsie
cd /home/r2d2/droid_ws/src/DROID/scripts
python real_main_dl_oopsie.py --check-only
```

With the DROID robot stack and OpenPI policy server running, launch:

```bash
python real_main_dl_oopsie.py --operator-name "Daniel Lawson"
```

Launching initializes DROID and resets the robot to its configured start pose.
The browser opens at http://localhost:5003. Submit a task instruction to begin
robot execution. Ctrl+C in the terminal ends an active rollout; otherwise it
ends at 600 actions. For a valid-length rollout, the terminal then asks:

```text
Keep this episode or discard it? [k/d]:
```

- `k` / `keep`: save the episode and videos, then annotate in the browser.
- `d` / `discard`: clear the buffered observations/actions/images, save no HDF5,
  videos, or annotation, and return the browser task to idle. Only the diagnostic
  entry in `rollouts.csv` remains.

There is no default choice; an empty or unrecognized answer asks again. After
keeping and annotating, or discarding, answer the terminal prompt to reset and
collect another episode, or exit. Ctrl+C at the keep/discard prompt exits without
saving the buffered episode.

The minimum valid episode is 15 recorded steps (one second at the nominal rate).
An earlier interruption produces only a diagnostic CSV entry, no episode.
A rollout error offers the same keep/discard choice for its completed prefix if
long enough. Keeping saves it and waits for annotation; discarding saves no
episode. Either choice then ends the session so you can investigate the error. Ctrl+C during annotation leaves any already saved
files on disk; they still need annotation before submission. Ctrl+C is handled
after an in-flight policy request or send-and-record pair completes, so it is
not an emergency stop. The annotation server runs in a separate process session
so Ctrl+C during a rollout leaves it available for the keep/discard decision and
annotation. If the server becomes unavailable when discarding, the data is still
discarded and the collector exits with an explanatory message.

## Multiple operators and output

Each launch creates a timestamped session under `/home/r2d2/oopsie/recordings`.
Each episode HDF5 stores `operator_name`. Daniel Lawson is the default; every
other operator must supply their own `--operator-name "Full Name"`.
The annotator defaults to that operator; use `--annotator-name "Full Name"`
when someone else labels the data. Close the previous collection session before
switching operators. An occupied annotation port is rejected to avoid reusing a
server configured for another annotator.

Sessions contain episode HDF5 files, three camera MP4s, annotation JSON files,
and `rollouts.csv` with status and measured command timing. Use
`--data-root-dir /absolute/path` to change the root. No upload runs automatically.

## Collection settings

Defaults match the existing local camera IDs and OpenPI endpoint:
`wss://green.iro.umontreal.ca:443`, left external camera + wrist for policy input,
and an open-loop horizon of 8. The returned chunk must contain at least 8 rows
of 8 finite action values. All three cameras and joint, Cartesian, and gripper
state are recorded. Tactile data is not acquired.

Commands target at most 15 Hz. Synchronous inference can lower the actual rate;
check the printed measurements and session `rollouts.csv`. Video and profile
use the nominal 15 Hz, so playback duration can differ from wall-clock duration.
Recorded actions are the clipped normalized DROID joint commands and binary
position-controlled gripper command actually sent to `env.step`.

Run `python real_main_dl_oopsie.py --help` for overrides. Local offline tests do
not verify physical robot behavior, camera availability, or server connectivity.
