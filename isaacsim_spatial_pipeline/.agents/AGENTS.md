Autonomous Runtime Policy

Codex may:
- Launch and stop Isaac Sim using the project’s approved Python entry point.
- Run Isaac Sim with a GUI or in headless mode.
- Load the approved existing USD stage.
- start, pause, and stop the simulation timeline programmatically.
- Control Spot through an approved deterministic trajectory controller.
- Launch and terminate project-owned ROS 2 nodes and launch files.
- Record run-scoped rosbag data when required by the experiment.
- Save map and evaluation artifacts under the designated output directory.
- Run static checks, runtime checks, and quantitative evaluation automatically.
- Repeat experiments using predefined parameter combinations.
- Update milestone documentation with results observed during the run.

Codex must:
- Apply a maximum runtime and experiment-count limit.
- Give every run a unique run ID.
- Store logs, bags, maps, configurations, and metrics under that run ID.
- Verify required topics and TF before beginning robot motion.
- Stop the experiment if required topics disappear, timestamps stall,
  transforms become invalid, or the robot leaves the permitted area.
- Clean up every process it starts.
- Distinguish measured evidence from visual assumptions.
- Report results honestly as PASS, WARN, or FAIL.

Codex must not:
- Fabricate /map, /odom, sensor topics, or TF transforms.
- overwrite the original USD stage.
- change Isaac prim names or the validated ROS 2 topic/frame contract
  unless the milestone explicitly requires it.
- install packages, modify system configuration, use sudo, or access files
  outside the project without approval.
- use unrestricted GUI clicking when a Python or ROS 2 interface exists.
- declare PASS solely because commands exited successfully.
- continue indefinitely after a failed or stalled experiment.