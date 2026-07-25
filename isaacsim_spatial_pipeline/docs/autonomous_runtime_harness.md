# Autonomous Runtime Harness

## Purpose

`scripts/100_run_autonomous_experiment.py` coordinates one bounded Isaac Sim
6.0 and ROS 2 Jazzy SLAM experiment from `config/autonomous_runtime.yaml`.
It reuses the approved runtime entry point, Milestone #9 SLAM launch, Milestone
#7 validator, Milestone #8 map utilities, and Milestone #9 scan diagnostic.

It does not publish synthetic sensor data, odometry, maps, or transforms.

## Milestone 9A Status

Static/unit tests, the dry-run gate, and the bounded live smoke gate pass.
The live smoke result is `WARN` only because its genuine saved map remains
below the Milestone 9B quality targets.

## Execution Sequence

1. Validate YAML, limits, paths, ROS packages, and NVIDIA GPU availability.
2. Create a unique, non-overwriting run directory.
3. Hash the source USD before execution.
4. Launch `scripts/10_run_sim.py` with headless auto-play and gated motion.
5. Launch the validated Milestone #9 stack.
6. Run the Milestone #7 topic, frame, TF-parent, and publisher validator.
7. Observe `/clock` directly.
8. Optionally start a run-scoped rosbag.
9. Trigger one deterministic trajectory.
10. Repeat fresh topic/TF validation and verify `/clock` advances.
11. Run the LaserScan diagnostic.
12. Save a real `/map`, validate its artifacts, and evaluate its quality.
13. Hash the source USD again, classify measured evidence, write the manifest
    and report, and stop every child process.

## Conservative Trajectory

The initial trajectory is simulation-only kinematic root-prim motion:

```text
speed: 0.30 m/s
x radius: 0.50 m
y radius: 0.25 m
maximum duration: 20 seconds
permitted relative x: [-0.75, 0.75] m
permitted relative y: [-0.50, 0.50] m
repeat: false
```

`scripts/10_run_sim.py` checks every path waypoint and commanded offset against
the permitted area. It stops moving after the first path or maximum duration.
The existing runtime exposes no collision callback suitable for this
orchestrator, so collision-triggered stopping is not claimed.

## Safety Limits

The YAML configuration provides:

```text
maximum experiment count
total wall-clock duration
startup timeout
topic timeout
trajectory timeout
shutdown timeout
watchdog window
maximum output size
```

Smoke mode additionally enforces:

```text
one experiment
one trajectory
at most 60 seconds of motion wall time
at most 120 seconds of Isaac runtime after runtime initialization
at most 500 MB of run output
no rosbag
```

Missing required topics, invalid frame IDs, TF parent conflicts, a stalled
simulation clock, controller exit, permitted-area violation, timeout, changed
source USD, or output-limit violation produces `FAIL` and cleanup.

## Output Layout

Each invocation creates:

```text
artifacts/autonomous_runs/run_<timestamp>_<pid>_<random>/
  effective_config.yaml
  manifest.json
  report.md
  logs/
  ros_logs/
  experiment_1_<name>/
    control/
    motion_status.json
    maps/
    bag/                 only when explicitly enabled
```

The run directory is created with `exist_ok=false`. Existing runs are never
overwritten. `manifest.json` and `report.md` are updated atomically as direct
evidence is collected.

## Commands

Static tests:

```bash
cd /home/aes/isaac_ws/isaacsim_spatial_pipeline
python3 -m unittest -v tests/test_autonomous_runtime.py
python3 -m compileall -q scripts launch tests
```

Dry-run validation:

```bash
python3 scripts/100_run_autonomous_experiment.py \
  --config config/autonomous_runtime.yaml \
  --dry-run
```

Bounded smoke test:

```bash
python3 scripts/100_run_autonomous_experiment.py \
  --config config/autonomous_runtime.yaml \
  --smoke
```

Full autonomous Milestone #9 experiment:

```bash
python3 /home/aes/isaac_ws/isaacsim_spatial_pipeline/scripts/100_run_autonomous_experiment.py \
  --config /home/aes/isaac_ws/isaacsim_spatial_pipeline/config/autonomous_runtime.yaml
```

Set `bag.enabled: true` only when a run-scoped bag is required. Bag topics and
the 500 MB total output limit remain explicit in the YAML.

## Directly Observed Validation

### Static and Unit Checks

Observed on 2026-07-25:

```text
Python compilation: PASS
YAML parsing and safety limits: PASS
Configuration-validation test: PASS
Process-cleanup test: PASS
Process-timeout test: PASS
Unique run-directory test: PASS
Evidence-classification test: PASS
Data-trajectory validation test: PASS
Topic/frame/artifact regression tests: PASS
Synthetic ATE/RPE tests: PASS
ROS packages pointcloud_to_laserscan, slam_toolbox, and tf2_ros: present
```

### Recorded Dry Run

Initial dry run:

```text
run_id: run_20260725T141103_3_9007ff12
result: PASS for the checks implemented at that point
processes_started: 0
source USD unchanged: true
```

The first live smoke showed that GPU availability was missing from the initial
dependency gate. The gate was corrected. Dry-run now validates configuration,
paths, ROS packages, safety caps, and cleanup separately from live readiness.
The current dry run records:

```text
run_id: run_20260725T152608_64233_b6c456ac
dry-run result: PASS
live gate: READY
Git revision: 6de870f4a9542c55c718040e95d0d015380731ac
source USD unchanged: true
cleanup confirmed: true
```

### Bounded Live Smoke Attempt

Successful bounded smoke:

```text
run_id: run_20260725T152626_64594_54de0973
result: WARN (map quality only)
motion complete: true
final topic/TF validation: PASS
scan diagnostic: PASS
PointCloud2 frequency: 1.960 Hz
finite LaserScan ranges/scan median: 168
finite-beam angular coverage median: 0.2324
saved-map artifact validation: PASS
known_ratio: 0.016017
occupied_cells: 96
cleanup confirmed: true
source USD unchanged: true
```

The smoke map improves occupied cells over the Milestone 8 baseline but does
not materially improve known coverage or meet the 0.10/500 Milestone 9B
targets. It is evidence for the harness live gate, not a selected map-quality
winner.

Earlier failed smoke evidence is preserved:

```text
run_id: run_20260725T141127_3_2b1a56eb
result: FAIL
source USD unchanged: true
output bytes: 47776
process cleanup: no owned Isaac/SLAM/orchestrator processes remained
```

Direct runtime evidence:

```text
Isaac Sim reported no usable CUDA device.
The loaded stage did not provide the required OmniLidar prim at
/World/spot_lidar_realsense/body/lidar_link/OS1/sensor.
Isaac exited before writing runtime-ready status.
No SLAM stack was launched.
No trajectory started.
No rosbag was recorded.
No map was saved.
```

That earlier run does not override the later bounded smoke evidence.
