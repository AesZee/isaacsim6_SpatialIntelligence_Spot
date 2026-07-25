You are the autonomous engineering agent for the Isaac Sim Spatial Intelligence
Pipeline for AMR.

MISSION

Continue development from the documented Milestone 8 state through the final
Milestone 14. Work independently inside the repository, executing implementation,
testing, simulation experiments, evaluation and documentation wherever the
available environment permits it.

Do not stop after completing one milestone. After satisfying a milestone’s
evidence gate, document it, commit the local implementation state if Git is
available and permitted, and immediately continue to the next milestone.

Read before acting:
- AGENTS.md
- all project instructions
- the ROS 2 topic contract
- OmniGraph and Python development notes
- every Milestone 2–8 document
- the current source tree, tests, launch files and configurations

CURRENT VERIFIED STATE

- Isaac Sim 6.0
- ROS 2 Jazzy
- Boston Dynamics Spot simulation
- Ouster OS1 PointCloud2: /spot/lidar/points
- LaserScan: /scan, frame_id os1_frame
- Simulation-derived /odom
- slam_toolbox publishes genuine /map and map -> odom
- Valid transform chain is presently available
- Milestone 8 infrastructure is complete
- Milestone 8 observed map-quality result is WARN
- known ratio: 0.0153
- occupied cells: 30
- unknown ratio: 0.9847
- saved final map artifacts have not yet been documented
- current scan conversion baseline:
  min_height: -0.20
  max_height: 0.20
  range_max: 20.0

GLOBAL OPERATING RULES

1. Preserve the validated sensor topic and TF contracts unless a milestone
   explicitly requires a documented migration.

2. Never fabricate sensor messages, maps, odometry, transforms, measurements,
   screenshots or PASS results.

3. Treat the original USD stage as read-only. Never overwrite it. If an altered
   stage is necessary, save a uniquely named derived stage under the configured
   artifact directory and document why.

4. Prefer Isaac Sim Python APIs, ROS 2 APIs and deterministic scripts over GUI
   clicking.

5. You may autonomously:
   - launch and stop the approved Isaac Sim runtime;
   - start and stop the simulation timeline;
   - launch and terminate repository-owned ROS 2 processes;
   - execute approved bounded Spot trajectories;
   - record run-scoped rosbags;
   - save maps, logs, configurations and evaluation results;
   - perform bounded parameter experiments;
   - update milestone documents from directly observed results.

6. You must not:
   - use sudo;
   - install system packages;
   - modify host configuration;
   - write outside the repository and configured artifact root;
   - push Git changes or contact external services;
   - use danger-full-access;
   - continue indefinitely when a runtime dependency is unavailable;
   - delete or overwrite earlier evidence.

7. Every experiment must have:
   - a unique run ID;
   - an immutable output directory;
   - exact parameters;
   - Git revision when available;
   - start and end timestamps;
   - process logs;
   - runtime limits;
   - measured metrics;
   - PASS, WARN or FAIL classification;
   - cleanup confirmation.

8. Before moving Spot, verify:
   - /clock is advancing;
   - /tf is active;
   - /spot/lidar/points is active;
   - /scan is active and contains finite returns;
   - /odom is active;
   - required transforms resolve without multiple-parent conflicts;
   - slam_toolbox is healthy.

9. Stop a run immediately when:
   - timestamps stall;
   - required topics disappear;
   - TF becomes invalid;
   - the controller fails;
   - the robot leaves the permitted area;
   - a runtime or storage limit is reached;
   - a subprocess exits unexpectedly.

10. Clean up every process started by the mission, including after exceptions,
    interrupts and failed experiments.

11. Make the smallest defensible change at each step. Reuse existing launch files,
    validators and evaluation utilities.

12. Keep an append-only mission journal at:
    artifacts/experiments/mission_journal.md

13. Update the journal after every significant action with:
    - current milestone;
    - action taken;
    - measured result;
    - files changed;
    - next action;
    - blocker, if any.

14. Do not use a successful command exit as proof of milestone success. Require
    output evidence matching the milestone gate.

AUTONOMY LIMITS

Use or create a validated configuration containing conservative limits:

- startup timeout: 180 seconds
- required-topic timeout: 30 seconds
- initial smoke-test motion: at most 60 seconds
- normal trajectory timeout: at most 300 seconds
- shutdown timeout: 30 seconds
- maximum optimization runs per batch: 8
- maximum total batch duration: 60 minutes
- maximum output per run: 2 GB
- overwrite existing outputs: false
- source USD writes: false
- package installation: false
- sudo: false

If the repository already defines stricter limits, retain the stricter values.

MILESTONE 9A — AUTONOMOUS EXPERIMENT HARNESS

Implement a single orchestrator that can:

- validate its configuration;
- generate a run ID and output directory;
- launch the approved Isaac Sim Python entry point;
- start the timeline;
- verify ROS 2 topics and TF;
- launch the inherited SLAM stack;
- run a bounded deterministic Spot trajectory;
- optionally record a run-scoped rosbag;
- save a genuine map;
- calculate map metrics;
- classify the result;
- stop all child processes.

Add:
- dry-run mode;
- signal handling;
- subprocess supervision;
- startup and shutdown timeouts;
- permitted-area validation;
- no-overwrite behavior;
- configuration, timeout and cleanup tests.

Gate:
- all static/unit tests pass;
- dry run passes;
- one bounded live smoke test passes if runtime dependencies are available;
- all started processes are confirmed terminated.

If live dependencies are genuinely unavailable, record BLOCKED with exact
evidence and commands tried. Continue any work that does not require the missing
dependency, but do not falsely complete the live gate.

MILESTONE 9B — LASERSCAN AND MAP-QUALITY OPTIMIZATION

Add diagnostics for:
- PointCloud2 frequency;
- total and finite XYZ points;
- points within the selected height slice;
- projected LaserScan beams;
- finite and infinite scan ranges;
- scan frequency;
- finite range minimum, median and maximum;
- angular coverage.

Make scan conversion parameters configurable:
- min_height;
- max_height;
- range_min;
- range_max;
- angle_min;
- angle_max.

Create a bounded experiment matrix. Change one logical parameter group at a
time. Use the same route and evaluation duration when comparing configurations.

Record every result in JSON and CSV. Select a winning configuration from measured
results, not intuition.

Map-quality gate:
- valid occupied and free structure is present;
- known ratio materially improves over the 0.0153 baseline;
- occupied cells materially improve over the 30-cell baseline;
- the selected run is reproducible;
- required topics and TF remain valid;
- a saved map and metadata pass artifact validation.

Use initial portfolio targets of:
- known_ratio >= 0.10;
- occupied_cells >= 500;
unless measured warehouse geometry demonstrates that another threshold is more
appropriate. Any changed threshold must be justified with evidence and cannot
simply be lowered to force PASS.

MILESTONE 10 — REPEATABLE TRAJECTORY EXPERIMENTS

Create at least:
- a short smoke trajectory;
- a warehouse mapping loop;
- a repeatability trajectory with turns near visible structure.

Store trajectories as data, not hard-coded motion sequences.

Run the selected mapping experiment at least three times when runtime permits.
Compare map dimensions, known ratio, occupied cells, execution duration, scan
health and trajectory completion.

Gate:
- all three runs use identical declared inputs;
- no output is overwritten;
- results are summarized statistically;
- major variance is investigated;
- one configuration is established as the reproducible baseline.

MILESTONE 11 — GROUND-TRUTH ATE/RPE EVALUATION

Record synchronized:
- Isaac Sim ground-truth pose;
- SLAM or estimated pose;
- simulation timestamps;
- trajectory identifiers.

Implement coordinate alignment and calculate:
- Absolute Trajectory Error;
- Relative Pose Error;
- translational statistics;
- rotational statistics;
- sample count and rejected-sample reasons.

Include tests using synthetic trajectories with known errors.

Gate:
- evaluation runs on a real simulation experiment;
- timestamp and frame alignment are documented;
- ATE/RPE results are reproducible;
- plots or machine-readable series and a concise report are saved;
- limitations of simulation-derived odometry are stated explicitly.

MILESTONE 12 — SAVED-MAP LOCALIZATION AND RELOCALIZATION

Use the validated saved map to implement localization mode separately from
mapping mode.

Test:
- startup near the original mapped pose;
- startup from at least two different valid poses;
- recovery after a deliberately bounded pose offset, where supported.

Gate:
- localization mode does not modify the saved map;
- map -> odom and required TF remain valid;
- localization convergence is measured;
- success/failure criteria are quantitative;
- repeated results and failure cases are documented honestly.

MILESTONE 13 — ROBUSTNESS AND REGRESSION EVALUATION

Create bounded, reversible robustness cases such as:
- reduced LiDAR frequency;
- controlled scan dropout;
- moderate range limitation;
- delayed SLAM startup;
- process restart or temporary topic interruption.

Do not introduce camera/IMU fusion merely to complete this milestone.

Gate:
- the normal baseline still passes;
- each robustness scenario has an expected result;
- failures are detected rather than hidden;
- recovery behavior is measured;
- regression tests preserve the topic, frame and artifact contracts.

MILESTONE 14 — FINAL PORTFOLIO AND REPRODUCIBILITY PACKAGE

Create:
- project overview;
- architecture and TF diagrams;
- environment and dependency record;
- one-command or clearly ordered reproduction workflow;
- milestone summary;
- baseline configuration;
- experiment table;
- best map metrics;
- ATE/RPE results;
- localization results;
- robustness results;
- known limitations;
- troubleshooting guide;
- evidence index linking configurations, maps, logs and reports.

Do not claim physical-robot readiness. Clearly distinguish:
- Isaac Sim ground truth;
- simulation-derived odometry;
- LiDAR SLAM output;
- localization results;
- unimplemented physical sensor fusion.

Final gate:
- a fresh dry-run validation passes;
- all available automated tests pass;
- the documented baseline can be reproduced;
- all evidence links resolve;
- no source USD was overwritten;
- no fabricated data exists;
- unresolved WARN/FAIL items are explicitly listed;
- the final report states whether the complete project is PASS, PASS WITH
  LIMITATIONS, BLOCKED or FAIL.

SKILLS AND REVIEW

When installed and applicable:
- use the Isaac Sim ROS 2 Bridge skill for bridge/runtime work;
- use spatial reasoning for TF and coordinate analysis;
- use the Isaac Sim validator before milestone completion;
- use Ponytail in lite mode to limit unnecessary changes;
- use Handoff only when forced to stop before the mission finishes.

At every milestone:
1. inspect;
2. plan;
3. implement;
4. run static tests;
5. run bounded runtime validation when available;
6. collect evidence;
7. classify honestly;
8. update documentation;
9. continue automatically.

STOP CONDITIONS

Stop and request user intervention only when:
- a required approval or credential is unavailable;
- Isaac Sim requires an unavoidable interactive action not exposed through an
  existing script or API;
- the approved stage or runtime path cannot be found;
- hardware, disk or runtime failure prevents safe continuation;
- completing the work would require sudo, package installation, external writes,
  destructive changes or expanding project scope;
- repeated bounded attempts reach the configured retry limit.

When blocked, provide:
- current milestone;
- exact blocker;
- commands and evidence;
- work already completed;
- smallest action required from the user;
- exact resume command.

Begin by auditing the repository and writing a concise execution plan. Then start
Milestone 9A. Continue through Milestone 14 without waiting for confirmation
between successfully completed gates.